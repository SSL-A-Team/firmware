#!/usr/bin/env python3
"""
Radio round-trip latency profiler for A-Team firmware.

Performs the standard multicast hello handshake to discover the robot,
then sends probe packets at a configurable rate and measures echo latency.

Probe packet layout (payload bytes, default 512):
  [0:4]   seq_num     uint32 LE  — monotonic counter
  [4:12]  tx_time     float64 LE — seconds since epoch (time.perf_counter base)
  [12:N]  padding     zeros

Metrics reported:
  RTT      — round-trip time (mean, stddev, min, max, p50, p95, p99)
  Jitter   — |RTT[i] - RTT[i-1]|  (mean and stddev of consecutive RTT delta)
  Drops    — packets with no echo within the inter-packet timeout window
  Ooo      — out-of-order echoes (seq arrives below highest seen)
  BurstLoss— max consecutive dropped packets

Usage examples:
  # 10 Hz for 10 seconds (default)
  python3 radio_latency.py

  # 100 Hz, run 500 packets
  python3 radio_latency.py --rate 100 --count 500

  # 1 kHz stress test for 30 seconds, write CSV
  python3 radio_latency.py --rate 1000 --duration 30 --csv out.csv
"""

import argparse
import csv
import datetime
import json
import socket
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

MULTICAST_IP = "224.4.20.69"
MULTICAST_PORT = 42069
LOCAL_PORT = 42069

# RadioHeader: crc32(4) + command_code(1) + _reserved(1) + data_length(2) = 8 bytes
RADIO_HEADER_FMT = "<IBB H"
RADIO_HEADER_SIZE = struct.calcsize(RADIO_HEADER_FMT)  # 8

# CommandCode values (uint8)
CC_HELLO_REQ = 21
CC_HELLO_RESP = 22

# HelloRequest: robot_id(1) + color(1) + bitfield(1) + reserved(1) + hashes(12) = 16 bytes
HELLO_REQUEST_SIZE = 16

# HelloResponse: ipv4(4) + port(2) = 6 bytes
HELLO_RESPONSE_FMT = "<4sH"
HELLO_RESPONSE_SIZE = struct.calcsize(HELLO_RESPONSE_FMT)  # 6

HELLO_REQ_PACKET_SIZE = RADIO_HEADER_SIZE + HELLO_REQUEST_SIZE   # 24
HELLO_RESP_PACKET_SIZE = RADIO_HEADER_SIZE + HELLO_RESPONSE_SIZE  # 14

# Probe packet header embedded in payload
PROBE_HEADER_FMT = "<Id"  # uint32 seq + float64 tx_time
PROBE_HEADER_SIZE = struct.calcsize(PROBE_HEADER_FMT)  # 12


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def local_ipv4() -> str:
    """Return the primary local IPv4 address."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]


def build_hello_resp(local_ip: str, port: int) -> bytes:
    ip_bytes = socket.inet_aton(local_ip)
    header = struct.pack(RADIO_HEADER_FMT, 0, CC_HELLO_RESP, 0, HELLO_RESPONSE_SIZE)
    resp = struct.pack(HELLO_RESPONSE_FMT, ip_bytes, port)
    return header + resp


def build_probe(seq: int, tx_time: float, payload_size: int) -> bytes:
    header = struct.pack(PROBE_HEADER_FMT, seq, tx_time)
    return header + bytes(payload_size - PROBE_HEADER_SIZE)


def parse_probe(data: bytes):
    """Return (seq, tx_time) from an echo'd probe packet."""
    if len(data) < PROBE_HEADER_SIZE:
        return None, None
    seq, tx_time = struct.unpack_from(PROBE_HEADER_FMT, data, 0)
    return seq, tx_time


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------

def discover_robot(sock: socket.socket, timeout: float = 30.0):
    """
    Wait for a CC_HELLO_REQ from the firmware on the multicast group.
    Returns (robot_addr, robot_src_port).
    Raises TimeoutError if nothing arrives within `timeout` seconds.
    """
    sock.settimeout(1.0)
    deadline = time.monotonic() + timeout
    print(f"Waiting for robot hello on {MULTICAST_IP}:{MULTICAST_PORT} ...")

    while time.monotonic() < deadline:
        try:
            data, src = sock.recvfrom(256)
        except TimeoutError:
            print("  ... waiting")
            continue

        if len(data) < HELLO_REQ_PACKET_SIZE:
            continue
        crc32, cmd, _res, data_len = struct.unpack_from(RADIO_HEADER_FMT, data, 0)
        if cmd == CC_HELLO_REQ:
            print(f"  Hello from {src[0]}:{src[1]}")
            return src

    raise TimeoutError("no hello received within timeout")


def do_hello_handshake(
    sock: socket.socket,
    local_ip: str,
    listen_port: int,
    hello_timeout: float = 30.0,
):
    """
    Full discovery + handshake. Returns robot_ip string.
    """
    robot_addr = discover_robot(sock, timeout=hello_timeout)
    robot_ip, robot_port = robot_addr

    resp = build_hello_resp(local_ip, listen_port)
    sock.sendto(resp, robot_addr)
    print(f"Sent hello response: local={local_ip}:{listen_port}")
    return robot_ip


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def percentile(data: List[float], p: float) -> float:
    if not data:
        return float("nan")
    sorted_data = sorted(data)
    idx = (len(sorted_data) - 1) * p / 100.0
    lo = int(idx)
    hi = lo + 1
    frac = idx - lo
    if hi >= len(sorted_data):
        return sorted_data[lo]
    return sorted_data[lo] * (1 - frac) + sorted_data[hi] * frac


def mean(data: List[float]) -> float:
    return sum(data) / len(data) if data else float("nan")


def stddev(data: List[float]) -> float:
    if len(data) < 2:
        return float("nan")
    m = mean(data)
    return (sum((x - m) ** 2 for x in data) / (len(data) - 1)) ** 0.5


@dataclass
class Stats:
    rtts: List[float] = field(default_factory=list)
    jitters: List[float] = field(default_factory=list)
    sent: int = 0
    echoed: int = 0
    dropped: int = 0
    ooo: int = 0
    max_burst_loss: int = 0
    _current_burst: int = 0
    _last_rtt: Optional[float] = None
    _highest_seq: int = -1

    def record_echo(self, seq: int, rtt: float):
        self.echoed += 1
        self.rtts.append(rtt)

        if self._last_rtt is not None:
            self.jitters.append(abs(rtt - self._last_rtt))
        self._last_rtt = rtt

        if seq <= self._highest_seq:
            self.ooo += 1
        else:
            self._highest_seq = seq

        self._current_burst = 0

    def record_drop(self):
        self.dropped += 1
        self._current_burst += 1
        self.max_burst_loss = max(self.max_burst_loss, self._current_burst)

    def to_dict(self, meta: Optional[Dict] = None) -> Dict:
        """Serialize summary stats to a JSON-serializable dict. All times in ms."""
        drop_pct = 100.0 * self.dropped / self.sent if self.sent > 0 else 0.0
        echo_pct = 100.0 * self.echoed / self.sent if self.sent > 0 else 0.0
        d: Dict = {
            "meta": meta or {},
            "packets": {
                "sent": self.sent,
                "echoed": self.echoed,
                "echoed_pct": round(echo_pct, 3),
                "dropped": self.dropped,
                "dropped_pct": round(drop_pct, 3),
                "out_of_order": self.ooo,
                "max_burst_loss": self.max_burst_loss,
            },
            "rtt_ms": {
                "mean":   round(mean(self.rtts) * 1e3, 4) if self.rtts else None,
                "stddev": round(stddev(self.rtts) * 1e3, 4) if self.rtts else None,
                "min":    round(min(self.rtts) * 1e3, 4) if self.rtts else None,
                "max":    round(max(self.rtts) * 1e3, 4) if self.rtts else None,
                "p50":    round(percentile(self.rtts, 50) * 1e3, 4) if self.rtts else None,
                "p95":    round(percentile(self.rtts, 95) * 1e3, 4) if self.rtts else None,
                "p99":    round(percentile(self.rtts, 99) * 1e3, 4) if self.rtts else None,
                "samples": [round(r * 1e3, 4) for r in self.rtts],
            },
            "jitter_ms": {
                "mean":   round(mean(self.jitters) * 1e3, 4) if self.jitters else None,
                "stddev": round(stddev(self.jitters) * 1e3, 4) if self.jitters else None,
                "max":    round(max(self.jitters) * 1e3, 4) if self.jitters else None,
                "samples": [round(j * 1e3, 4) for j in self.jitters],
            },
        }
        return d

    def print_report(self):
        drop_pct = 100.0 * self.dropped / self.sent if self.sent > 0 else 0.0
        echo_pct = 100.0 * self.echoed / self.sent if self.sent > 0 else 0.0

        print("\n" + "=" * 60)
        print(f"  Packets:  sent={self.sent}  echoed={self.echoed} ({echo_pct:.1f}%)"
              f"  dropped={self.dropped} ({drop_pct:.1f}%)")
        print(f"  OOO:      {self.ooo}  |  Max burst loss: {self.max_burst_loss}")

        if self.rtts:
            print(f"\n  RTT (ms):")
            print(f"    mean  = {mean(self.rtts)*1e3:8.3f}")
            print(f"    stddev= {stddev(self.rtts)*1e3:8.3f}")
            print(f"    min   = {min(self.rtts)*1e3:8.3f}")
            print(f"    max   = {max(self.rtts)*1e3:8.3f}")
            print(f"    p50   = {percentile(self.rtts, 50)*1e3:8.3f}")
            print(f"    p95   = {percentile(self.rtts, 95)*1e3:8.3f}")
            print(f"    p99   = {percentile(self.rtts, 99)*1e3:8.3f}")
        else:
            print("\n  No RTT samples collected.")

        if self.jitters:
            print(f"\n  Jitter (ms)  [|RTT[i] - RTT[i-1]|]:")
            print(f"    mean  = {mean(self.jitters)*1e3:8.3f}")
            print(f"    stddev= {stddev(self.jitters)*1e3:8.3f}")
            print(f"    max   = {max(self.jitters)*1e3:8.3f}")

        print("=" * 60)


# ---------------------------------------------------------------------------
# Probe loop
# ---------------------------------------------------------------------------

def run_probe_loop(
    sock: socket.socket,
    robot_ip: str,
    robot_port: int,
    rate_hz: float,
    payload_size: int,
    max_count: Optional[int],
    duration_s: Optional[float],
    csv_path: Optional[str],
    stats: Stats,
):
    interval = 1.0 / rate_hz
    # Drop window: wait up to 3 inter-packet intervals for an echo before marking as dropped.
    # At high rates (>100 Hz) use a fixed 100 ms floor so we don't drop prematurely.
    drop_timeout = max(3.0 * interval, 0.1)

    sock.settimeout(drop_timeout)

    pending: dict = {}  # seq -> tx_time
    seq = 0
    start_time = time.perf_counter()
    deadline = start_time + duration_s if duration_s else None

    csv_writer = None
    csv_file = None
    if csv_path:
        csv_file = open(csv_path, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["seq", "tx_time", "rx_time", "rtt_ms", "dropped"])

    print(f"\nProbing {robot_ip}:{robot_port} at {rate_hz:.1f} Hz "
          f"| payload={payload_size}B | drop_timeout={drop_timeout*1e3:.0f}ms")
    if max_count:
        print(f"  count limit: {max_count}")
    if duration_s:
        print(f"  duration limit: {duration_s:.1f}s")
    print("  Ctrl-C to stop early.\n")

    next_tx = time.perf_counter()

    try:
        while True:
            now = time.perf_counter()

            if deadline and now >= deadline:
                break
            if max_count and stats.sent >= max_count:
                break

            # Send next probe if it's time
            if now >= next_tx:
                tx_time = time.perf_counter()
                pkt = build_probe(seq, tx_time, payload_size)
                try:
                    sock.sendto(pkt, (robot_ip, robot_port))
                    pending[seq] = tx_time
                    stats.sent += 1
                    if stats.sent % 100 == 0:
                        print(f"  sent={stats.sent} echoed={stats.echoed} "
                              f"dropped={stats.dropped} ooo={stats.ooo}")
                except OSError as e:
                    print(f"  send error: {e}", file=sys.stderr)
                seq += 1
                next_tx += interval

            # Try to receive an echo
            try:
                data, _src = sock.recvfrom(payload_size + 64)
                rx_time = time.perf_counter()
                rx_seq, tx_time_embedded = parse_probe(data)

                if rx_seq is not None and tx_time_embedded is not None:
                    rtt = rx_time - tx_time_embedded
                    stats.record_echo(rx_seq, rtt)
                    pending.pop(rx_seq, None)

                    if csv_writer:
                        csv_writer.writerow(
                            [rx_seq, tx_time_embedded, rx_time, rtt * 1e3, 0]
                        )

            except TimeoutError:
                # Drop all pending packets older than drop_timeout
                now2 = time.perf_counter()
                expired = [s for s, t in pending.items() if now2 - t > drop_timeout]
                for s in expired:
                    stats.record_drop()
                    del pending[s]
                    if csv_writer:
                        csv_writer.writerow([s, pending.get(s, 0), 0, 0, 1])

    except KeyboardInterrupt:
        print("\n  Interrupted.")

    # Mark any remaining pending as dropped
    for s in list(pending.keys()):
        stats.record_drop()
        if csv_writer:
            csv_writer.writerow([s, pending[s], 0, 0, 1])

    if csv_file:
        csv_file.close()
        print(f"  Results written to {csv_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="A-Team radio round-trip latency profiler",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        metavar="HZ",
        help="Probe packet send rate in Hz (1-1000, default 10)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        metavar="SECS",
        help="Stop after this many seconds (default: run until --count or Ctrl-C)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=None,
        metavar="N",
        help="Stop after sending N packets (default: unlimited)",
    )
    parser.add_argument(
        "--payload-size",
        type=int,
        default=512,
        metavar="BYTES",
        help="UDP payload size in bytes (default 512, matches both firmware variants)",
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        metavar="PATH",
        help="Write per-packet results to a CSV file",
    )
    parser.add_argument(
        "--json",
        type=str,
        default=None,
        metavar="PATH",
        help="Write summarized stats to a JSON file (for visualizers)",
    )
    parser.add_argument(
        "--hello-timeout",
        type=float,
        default=30.0,
        metavar="SECS",
        help="Seconds to wait for firmware hello (default 30)",
    )
    args = parser.parse_args()

    # Validate
    if not (1.0 <= args.rate <= 1000.0):
        parser.error("--rate must be between 1 and 1000 Hz")
    if args.payload_size < PROBE_HEADER_SIZE:
        parser.error(f"--payload-size must be >= {PROBE_HEADER_SIZE} (probe header size)")
    if args.payload_size > 65000:
        parser.error("--payload-size must be <= 65000")
    if args.duration is None and args.count is None:
        # Default: 10 seconds
        args.duration = 10.0
        print(f"No --duration or --count specified; defaulting to {args.duration:.0f}s")

    local_ip = local_ipv4()
    print(f"Local IP: {local_ip}")

    # Set up socket: bind to LOCAL_PORT, join multicast
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass  # not available on Windows
    sock.bind(("", LOCAL_PORT))

    # Join multicast group
    mreq = socket.inet_aton(MULTICAST_IP) + socket.inet_aton("0.0.0.0")
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    try:
        robot_ip = do_hello_handshake(sock, local_ip, LOCAL_PORT, hello_timeout=args.hello_timeout)
    except TimeoutError as e:
        print(f"Discovery failed: {e}", file=sys.stderr)
        sock.close()
        sys.exit(1)

    stats = Stats()
    run_probe_loop(
        sock=sock,
        robot_ip=robot_ip,
        robot_port=LOCAL_PORT,
        rate_hz=args.rate,
        payload_size=args.payload_size,
        max_count=args.count,
        duration_s=args.duration,
        csv_path=args.csv,
        stats=stats,
    )

    stats.print_report()

    if args.json:
        meta = {
            "timestamp": datetime.datetime.now().isoformat(),
            "rate_hz": args.rate,
            "payload_size_bytes": args.payload_size,
            "duration_s": args.duration,
            "count_limit": args.count,
            "robot_ip": robot_ip,
        }
        with open(args.json, "w") as f:
            json.dump(stats.to_dict(meta), f, indent=2)
        print(f"Summary written to {args.json}")

    sock.close()


if __name__ == "__main__":
    main()
