#!/usr/bin/env python3
"""
Radio communications reliability tester for SSL A-Team robots.

Performs multicast discovery, then sends configurable bursts of control packets
to stress-test radio link reliability. Supports injecting invalid packets,
duplicate packets, and variable inter-packet delays to emulate network unreliability.

Usage:
    python3 coms_reliability.py [OPTIONS]

Examples:
    # Discover robot and send packets at 100 Hz until Ctrl+C
    python3 coms_reliability.py --robot-id 1

    # Send exactly 100 packets (useful for automated testing)
    python3 coms_reliability.py --robot-id 1 --count 100

    # Burst mode: groups of 5 packets with random 50-500 ms gaps, run forever
    python3 coms_reliability.py --robot-id 1 --burst-size 5 --burst-gap-min 0.05 --burst-gap-max 0.5

    # Inject 10% invalid + 5% duplicate packets until Ctrl+C
    python3 coms_reliability.py --robot-id 1 --invalid-rate 0.1 --duplicate-rate 0.05

    # Connect directly without multicast discovery, stop after 50 packets
    python3 coms_reliability.py --robot-ip 192.168.1.10 --robot-port 42069 --robot-id 1 --count 50

    # Bound discovery to 5 attempts (for scripted environments)
    python3 coms_reliability.py --robot-id 1 --discovery-retries 5 --count 100
"""

import argparse
import json
import random
import socket
import struct
import sys
import time
import zlib
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Dict, Optional, Tuple


# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

MULTICAST_IP = "224.4.20.69"
MULTICAST_PORT = 42069
LOCAL_PORT = 42069

PROTOCOL_VERSION_MAJOR = 0
PROTOCOL_VERSION_MINOR = 1

RADIO_PACKET_TOTAL_SIZE = 408
RADIO_HEADER_SIZE = 12      # crc32(4) + major(2) + minor(2) + cmd(1) + data_len(2) + reserved(1)
RADIO_DATA_MAX_SIZE = RADIO_PACKET_TOTAL_SIZE - RADIO_HEADER_SIZE  # 396


class CommandCode(IntEnum):
    CC_ACK                      = 1
    CC_NACK                     = 2
    CC_GOODBYE                  = 3
    CC_KEEPALIVE                = 4
    CC_HELLO_REQ                = 101
    CC_TELEMETRY                = 102
    CC_CONTROL_DEBUG_TELEMETRY  = 103
    CC_ROBOT_PARAMETER_COMMAND  = 104
    CC_ERROR_TELEMETRY          = 105
    CC_CONTROL                  = 201
    CC_HELLO_RESP               = 202


class TeamColor(IntEnum):
    TC_YELLOW = 0
    TC_BLUE   = 1


class KickRequest(IntEnum):
    NO_KICK    = 0
    KICK       = 1
    CHIP_KICK  = 2


# ---------------------------------------------------------------------------
# Packet building helpers
# ---------------------------------------------------------------------------

def _compute_crc32(data: bytes) -> int:
    """CRC32 over the packet bytes with the crc32 field zeroed."""
    return zlib.crc32(data) & 0xFFFFFFFF


def build_packet(command_code: int, data: bytes) -> bytes:
    """
    Build a RadioPacket at exact length (no padding).

    Layout (little-endian):
      u32  crc32         (computed over packet with this field = 0)
      u16  major_version
      u16  minor_version
      u8   command_code
      u16  data_length
      u8   _reserved
      u8[] data
    """
    assert len(data) <= RADIO_DATA_MAX_SIZE, f"data too large: {len(data)} > {RADIO_DATA_MAX_SIZE}"

    header_no_crc = struct.pack(
        "<xxxx HH B H x",          # 4 bytes reserved for crc32, then rest of header
        PROTOCOL_VERSION_MAJOR,
        PROTOCOL_VERSION_MINOR,
        int(command_code),
        len(data),
    )
    payload = header_no_crc + data

    crc = _compute_crc32(payload)
    return struct.pack("<I", crc) + payload[4:]


def build_hello_request(robot_id: int, color: TeamColor) -> bytes:
    """CC_HELLO_REQ payload: robot_id (u8) + color (u8)."""
    data = struct.pack("<BB", robot_id, int(color))
    return build_packet(CommandCode.CC_HELLO_REQ, data)


def build_hello_response(local_ip: str, local_port: int) -> bytes:
    """CC_HELLO_RESP payload: ipv4[4] + port(u16)."""
    ip_bytes = bytes(int(b) for b in local_ip.split("."))
    data = ip_bytes + struct.pack("<H", local_port)
    return build_packet(CommandCode.CC_HELLO_RESP, data)


def build_control_packet(
    *,
    body_x: float = 0.0,
    body_y: float = 0.0,
    body_w: float = 0.0,
    kick_vel: float = 0.0,
    dribbler_speed: float = 0.0,
    dribbler_multiplier: int = 0,
    kick_request: KickRequest = KickRequest.NO_KICK,
    emergency_stop: bool = False,
    body_twist_control_enabled: bool = True,
) -> bytes:
    """
    Build a CC_CONTROL packet with BasicControl payload (40 bytes).

    Bitfield (u32):
      bit 0:  request_shutdown
      bit 1:  reboot_robot
      bit 2:  game_state_in_stop
      bit 3:  emergency_stop
      bit 4:  body_pose_control_enabled
      bit 5:  body_twist_control_enabled
      bit 6:  body_accel_control_enabled
      bit 7:  wheel_vel_control_enabled
      bit 8:  wheel_torque_control_enabled
      bit 9:  vision_update
      bits 10-17: dribbler_multiplier (8 bits)
      bits 18-23: _reserved (6 bits)
      bits 24-31: play_song (8 bits)

    Floats (7 x f32): pose_x_vision, pose_y_vision, pose_z_vision,
                       x_linear_cmd, y_linear_cmd, z_linear_cmd,
                       kick_vel, dribbler_speed

    KickRequest (u8), padded to align.
    """
    bitfield = 0
    if emergency_stop:
        bitfield |= (1 << 3)
    if body_twist_control_enabled:
        bitfield |= (1 << 5)
    bitfield |= ((dribbler_multiplier & 0xFF) << 10)

    # 40 bytes: u32 bitfield + 8 floats + KickRequest(u8) + 3 pad bytes
    data = struct.pack(
        "<I 8f B 3x",
        bitfield,
        0.0,           # pose_x_vision
        0.0,           # pose_y_vision
        0.0,           # pose_z_vision
        body_x,        # x_linear_cmd
        body_y,        # y_linear_cmd
        body_w,        # z_linear_cmd
        kick_vel,
        dribbler_speed,
        int(kick_request),
    )
    assert len(data) == 40, f"BasicControl size mismatch: {len(data)}"
    return build_packet(CommandCode.CC_CONTROL, data)


# ---------------------------------------------------------------------------
# Invalid / malformed packet generators
# ---------------------------------------------------------------------------

def make_bad_crc(packet: bytes) -> bytes:
    """Flip the CRC so the robot will reject it."""
    crc = struct.unpack_from("<I", packet, 0)[0]
    bad_crc = crc ^ 0xDEADBEEF
    return struct.pack("<I", bad_crc) + packet[4:]


def make_bad_command_code(packet: bytes) -> bytes:
    """Replace command_code with 0xFF (undefined)."""
    return packet[:8] + b'\xFF' + packet[9:]


def make_wrong_length(packet: bytes) -> bytes:
    """Set data_length field to a value inconsistent with the actual data."""
    bad_len = struct.unpack_from("<H", packet, 9)[0] ^ 0x00FF
    return packet[:9] + struct.pack("<H", bad_len) + packet[11:]


def make_truncated(packet: bytes) -> bytes:
    """Return a packet that is shorter than RADIO_PACKET_TOTAL_SIZE."""
    trunc = max(1, len(packet) // 2)
    return packet[:trunc]


def make_random_garbage() -> bytes:
    """Completely random bytes at the full packet size."""
    return bytes(random.getrandbits(8) for _ in range(RADIO_PACKET_TOTAL_SIZE))


INVALID_MUTATIONS = [
    make_bad_crc,
    make_bad_command_code,
    make_wrong_length,
    make_truncated,
    make_random_garbage,
]


def make_invalid_packet(base_packet: Optional[bytes] = None) -> bytes:
    """Apply a random mutation to produce an invalid packet."""
    mutation = random.choice(INVALID_MUTATIONS)
    if mutation is make_random_garbage:
        return make_random_garbage()
    if base_packet is None:
        base_packet = build_control_packet()
    return mutation(base_packet)


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------


def _get_local_ip(target_ip: str) -> str:
    """Return the local IP on the interface that would route to target_ip."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect((target_ip, MULTICAST_PORT))
            return s.getsockname()[0]
        except OSError:
            return "127.0.0.1"


def discover_robot(
    robot_id: int,
    color: TeamColor,
    timeout: float = 1.0,
    max_retries: Optional[int] = None,
) -> Optional[Tuple[str, int]]:
    """
    Listen for CC_HELLO_REQ via multicast and respond with CC_HELLO_RESP.

    Joins the multicast group and waits for a robot to broadcast CC_HELLO_REQ,
    then replies unicast with CC_HELLO_RESP containing the local IP and port.
    Returns the robot's (ip, port) address, or None if all attempts exhausted.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(timeout)
    sock.bind(("", MULTICAST_PORT))

    # Join the multicast group to receive robot broadcasts
    mreq = struct.pack("4sL", socket.inet_aton(MULTICAST_IP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    infinite = max_retries is None or max_retries <= 0
    limit_str = "inf" if infinite else str(max_retries)

    try:
        attempt = 0
        while True:
            attempt += 1
            print(f"[discovery] Listening for CC_HELLO_REQ on {MULTICAST_IP}:{MULTICAST_PORT} "
                  f"(attempt {attempt}/{limit_str})")

            try:
                raw, addr = sock.recvfrom(RADIO_PACKET_TOTAL_SIZE)
            except socket.timeout:
                print(f"[discovery] Timeout waiting for CC_HELLO_REQ")
                if not infinite and attempt >= max_retries:
                    return None
                continue

            if len(raw) < RADIO_HEADER_SIZE:
                print(f"[discovery] Packet too short ({len(raw)} bytes), ignoring")
                if not infinite and attempt >= max_retries:
                    return None
                continue

            cmd = raw[8]
            if cmd != CommandCode.CC_HELLO_REQ:
                print(f"[discovery] Unexpected command code 0x{cmd:02X}, ignoring")
                if not infinite and attempt >= max_retries:
                    return None
                continue

            data_len = struct.unpack_from("<H", raw, 9)[0]
            if data_len < 2:
                print(f"[discovery] CC_HELLO_REQ data too short ({data_len} bytes)")
                if not infinite and attempt >= max_retries:
                    return None
                continue

            # HelloRequest: robot_id (u8) + color (u8)
            req_robot_id = raw[RADIO_HEADER_SIZE]
            if req_robot_id != robot_id:
                print(f"[discovery] CC_HELLO_REQ for robot {req_robot_id}, expected {robot_id}, ignoring")
                if not infinite and attempt >= max_retries:
                    return None
                continue

            robot_ip, robot_port = addr[0], addr[1]
            print(f"[discovery] Got CC_HELLO_REQ from robot {req_robot_id} at {robot_ip}:{robot_port}")

            # Reply unicast to the sender with our local IP on the interface
            # that routes to the robot (mirrors radio_bridge GetClosestIpAddress)
            local_ip = _get_local_ip(robot_ip)
            resp_pkt = build_hello_response(local_ip, LOCAL_PORT)
            sock.sendto(resp_pkt, (robot_ip, robot_port))
            print(f"[discovery] Sent CC_HELLO_RESP to {robot_ip}:{robot_port} (local {local_ip}:{LOCAL_PORT})")

            return robot_ip, robot_port

    except KeyboardInterrupt:
        print("\n[discovery] Interrupted by user")
        return None
    finally:
        sock.close()


# ---------------------------------------------------------------------------
# Statistics tracker
# ---------------------------------------------------------------------------

@dataclass
class Stats:
    sent_total: int = 0
    sent_valid: int = 0
    sent_invalid: int = 0
    sent_duplicate: int = 0
    send_errors: int = 0
    start_time: float = field(default_factory=time.monotonic)

    def report(self) -> None:
        elapsed = time.monotonic() - self.start_time
        rate = self.sent_total / elapsed if elapsed > 0 else 0.0
        print(
            f"\n--- Statistics ---\n"
            f"  Duration:       {elapsed:.2f}s\n"
            f"  Total sent:     {self.sent_total}\n"
            f"  Valid packets:  {self.sent_valid}\n"
            f"  Invalid:        {self.sent_invalid}\n"
            f"  Duplicates:     {self.sent_duplicate}\n"
            f"  Send errors:    {self.send_errors}\n"
            f"  Avg rate:       {rate:.1f} pkt/s\n"
        )


# ---------------------------------------------------------------------------
# Main sender loop
# ---------------------------------------------------------------------------

def run_sender(
    robot_addr: Tuple[str, int],
    *,
    count: Optional[int],
    rate_hz: float,
    burst_size: int,
    burst_gap_min: float,
    burst_gap_max: float,
    intra_burst_delay: float,
    invalid_rate: float,
    duplicate_rate: float,
    control_padding: int,
    verbose: bool,
) -> Stats:
    """
    Send packets to `robot_addr`.

    count=None runs indefinitely until Ctrl+C; pass an integer to stop after
    that many packet slots (useful for automated testing).

    Burst behaviour:
        - Packets are grouped into bursts of `burst_size`.
        - Within a burst, packets are sent with `intra_burst_delay` seconds between them.
        - Between bursts, a random pause in [burst_gap_min, burst_gap_max] seconds is inserted.
        - If burst_size == 1 and burst_gap_min == burst_gap_max, this degrades to steady-rate.

    Injection:
        - Each packet slot is independently rolled against `invalid_rate` and `duplicate_rate`.
        - A duplicate sends the same bytes twice (back-to-back) before moving on.
        - An invalid packet replaces the valid packet entirely.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", LOCAL_PORT))
    stats = Stats()
    infinite = count is None

    def _send(payload: bytes, label: str = "pkt") -> bool:
        try:
            sock.sendto(payload, robot_addr)
            if verbose:
                cmd = payload[8] if len(payload) > 8 else 0xFF
                print(f"  -> {label} cmd=0x{cmd:02X} len={len(payload)}")
            return True
        except OSError as e:
            print(f"  [send error] {e}")
            stats.send_errors += 1
            return False

    try:
        base_delay = 1.0 / rate_hz if rate_hz > 0 else 0.0
        sent = 0
        burst_count = 0

        while infinite or sent < count:
            burst_count += 1
            burst_actual = burst_size if infinite else min(burst_size, count - sent)
            if verbose:
                print(f"[burst {burst_count}] sending {burst_actual} packet(s)")

            for i in range(burst_actual):
                valid_pkt = build_control_packet(body_twist_control_enabled=True)
                valid_pkt += b'\x00' * control_padding

                # Decide what to actually send
                roll_invalid = random.random() < invalid_rate
                roll_duplicate = random.random() < duplicate_rate

                if roll_invalid:
                    pkt = make_invalid_packet(valid_pkt)
                    _send(pkt, "INVALID")
                    stats.sent_invalid += 1
                else:
                    _send(valid_pkt, "valid")
                    stats.sent_valid += 1

                    if roll_duplicate:
                        _send(valid_pkt, "DUPLICATE")
                        stats.sent_duplicate += 1

                stats.sent_total += 1
                sent += 1

                if i < burst_actual - 1 and intra_burst_delay > 0:
                    time.sleep(intra_burst_delay)

            # Inter-burst gap
            more_to_send = infinite or sent < count
            if more_to_send:
                gap = random.uniform(burst_gap_min, burst_gap_max)
                if verbose:
                    print(f"[burst {burst_count}] gap {gap*1000:.1f} ms")
                time.sleep(gap)
            elif base_delay > 0 and burst_size == 1:
                # Steady-rate: honour the base delay after each single packet
                time.sleep(base_delay)

    except KeyboardInterrupt:
        print("\n[interrupted]")
    finally:
        sock.close()

    return stats


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="SSL A-Team radio communications reliability tester",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    p.add_argument("--config", metavar="FILE",
                   help="Load settings from a JSON file (CLI flags override file values)")
    p.add_argument("--dump-config", action="store_true",
                   help="Print the active configuration as JSON and exit")

    disc = p.add_argument_group("Discovery")
    disc.add_argument("--robot-id", type=int, default=0, metavar="ID",
                      help="Robot ID for CC_HELLO_REQ (default: 0)")
    disc.add_argument("--color", choices=["yellow", "blue"], default="yellow",
                      help="Team color for CC_HELLO_REQ (default: yellow)")
    disc.add_argument("--discovery-timeout", type=float, default=1.0, metavar="SEC",
                      help="Timeout per CC_HELLO_RESP wait in seconds (default: 1.0)")
    disc.add_argument("--discovery-retries", type=int, default=0, metavar="N",
                      help="Max CC_HELLO_REQ attempts before giving up (default: 0 = infinite)")
    disc.add_argument("--robot-ip", type=str, default=None, metavar="IP",
                      help="Skip discovery and connect directly to this IP")
    disc.add_argument("--robot-port", type=int, default=MULTICAST_PORT, metavar="PORT",
                      help=f"Robot unicast port when using --robot-ip (default: {MULTICAST_PORT})")

    send = p.add_argument_group("Sending")
    send.add_argument("--count", type=int, default=None, metavar="N",
                      help="Stop after N packet slots (default: run until Ctrl+C)")
    send.add_argument("--rate", type=float, default=100.0, metavar="HZ",
                      help="Base packet rate in Hz for steady-rate mode (default: 100.0)")

    burst = p.add_argument_group("Burst pattern")
    burst.add_argument("--burst-size", type=int, default=1, metavar="N",
                       help="Packets per burst (default: 1 = steady rate)")
    burst.add_argument("--burst-gap-min", type=float, default=None, metavar="SEC",
                       help="Minimum inter-burst gap in seconds (default: 1/rate)")
    burst.add_argument("--burst-gap-max", type=float, default=None, metavar="SEC",
                       help="Maximum inter-burst gap in seconds (default: 1/rate)")
    burst.add_argument("--intra-burst-delay", type=float, default=0.002, metavar="SEC",
                       help="Delay between packets within a burst in seconds (default: 0.002)")

    inject = p.add_argument_group("Fault injection")
    inject.add_argument("--invalid-rate", type=float, default=0.0, metavar="FRAC",
                        help="Fraction [0,1] of slots that send an invalid packet (default: 0.0)")
    inject.add_argument("--duplicate-rate", type=float, default=0.0, metavar="FRAC",
                        help="Fraction [0,1] of valid slots that are also duplicated (default: 0.0)")
    inject.add_argument("--control-padding", type=int, default=0, metavar="BYTES",
                        help="Extra zero bytes appended to each control packet (default: 0)")

    p.add_argument("--verbose", "-v", action="store_true",
                   help="Print each packet send")

    return p


def _args_to_config(args: argparse.Namespace) -> Dict[str, Any]:
    """Return the serialisable subset of an argparse namespace (excludes meta-flags)."""
    d = vars(args).copy()
    d.pop("config", None)
    d.pop("dump_config", None)
    return d


def parse_args() -> argparse.Namespace:
    p = _build_parser()

    # First pass: find --config without erroring on unknown flags.
    partial, _ = p.parse_known_args()

    if partial.config is not None:
        try:
            with open(partial.config) as f:
                file_cfg: Dict[str, Any] = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            p.error(f"cannot load config file '{partial.config}': {e}")

        # Validate keys against known destinations.
        known_dests = {a.dest for a in p._actions}
        unknown = set(file_cfg) - known_dests
        if unknown:
            p.error(f"unknown key(s) in config file: {', '.join(sorted(unknown))}")

        # Apply file values as new defaults so CLI flags still win.
        p.set_defaults(**file_cfg)

    return p.parse_args()


def main() -> None:
    args = parse_args()

    if args.dump_config:
        print(json.dumps(_args_to_config(args), indent=2))
        sys.exit(0)

    # Resolve robot address
    if args.robot_ip:
        robot_addr = (args.robot_ip, args.robot_port)
        print(f"[config] Using direct address {robot_addr[0]}:{robot_addr[1]}")
    else:
        color = TeamColor.TC_YELLOW if args.color == "yellow" else TeamColor.TC_BLUE
        result = discover_robot(
            robot_id=args.robot_id,
            color=color,
            timeout=args.discovery_timeout,
            max_retries=args.discovery_retries,
        )
        if result is None:
            print("[error] Discovery failed — no CC_HELLO_RESP received. "
                  "Use --robot-ip to skip discovery.")
            return
        robot_addr = result

    # Resolve burst gap defaults from rate
    base_delay = 1.0 / args.rate if args.rate > 0 else 0.0
    burst_gap_min = args.burst_gap_min if args.burst_gap_min is not None else base_delay
    burst_gap_max = args.burst_gap_max if args.burst_gap_max is not None else base_delay

    time.sleep(0.1)

    count_str = str(args.count) if args.count is not None else "inf"
    print(
        f"\n[config] Sending {count_str} packet(s) to {robot_addr[0]}:{robot_addr[1]}\n"
        f"         rate={args.rate} Hz  burst_size={args.burst_size}\n"
        f"         burst_gap=[{burst_gap_min*1000:.1f}, {burst_gap_max*1000:.1f}] ms"
        f"  intra={args.intra_burst_delay*1000:.1f} ms\n"
        f"         invalid_rate={args.invalid_rate:.0%}"
        f"  duplicate_rate={args.duplicate_rate:.0%}\n"
    )

    stats = run_sender(
        robot_addr,
        count=args.count,
        rate_hz=args.rate,
        burst_size=args.burst_size,
        burst_gap_min=burst_gap_min,
        burst_gap_max=burst_gap_max,
        intra_burst_delay=args.intra_burst_delay,
        invalid_rate=args.invalid_rate,
        duplicate_rate=args.duplicate_rate,
        control_padding=args.control_padding,
        verbose=args.verbose,
    )

    stats.report()


if __name__ == "__main__":
    main()
