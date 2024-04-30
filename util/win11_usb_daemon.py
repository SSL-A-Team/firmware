import dataclasses
import json
import shutil
import signal
import subprocess
import time
import traceback

WSL_CMD_LITE_WAKEUP = ["wsl", "--exec", "echo", "hi"]
WSL_CMD_LSUSB = ["wsl", "--exec", "lsusb"]

USBIPD_CMD_LIST = ["usbipd", "list"]
USBIPD_CMD_STATE = ["usbipd", "state"]
USBIPD_CMD_BIND_BUSID = ["usbipd", "bind", "--busid", "ID"]
USBIPD_CMD_ATTACH_BUSID = ["usbipd", "attach", "--wsl", "--busid", "ID"]
USBIPD_CMD_DETACH_BUSID = ["usbipd", "detach", "--busid", "ID"]


ST_LINK_DESCR_KEY = "ST-Link Debug"
DEV_KEY = "Devices"

running = True
current_state = None

@dataclasses.dataclass
class Device:
    DEV_BUSID_KEY = "BusId"
    DEV_CLIENT_IP_KEY = "ClientIPAddress"
    DEV_DESC_KEY = "Description"
    DEV_INST_ID_KEY = "InstanceId"
    DEV_FORCED_KEY = "IsForced"
    DEV_PER_GUID_KEY = "PersistedGuid"
    DEV_STUB_INST_ID_KEY = "StubInstanceId"

    bus_id: str
    client_ip_address: str
    description: str
    instance_id: str
    is_forced: bool
    persisted_guid: str
    stub_instance_id: str

    @staticmethod
    def from_usbipd_dict(dict: dict):
        dev = Device(
            dict[Device.DEV_BUSID_KEY],
            dict[Device.DEV_CLIENT_IP_KEY],
            dict[Device.DEV_DESC_KEY],
            dict[Device.DEV_INST_ID_KEY],
            dict[Device.DEV_FORCED_KEY],
            dict[Device.DEV_PER_GUID_KEY],
            dict[Device.DEV_STUB_INST_ID_KEY])

        return dev
    
    def __hash__(self):
        hashed = hash(self.instance_id)
        return hashed

    
def check_usbipd():
    return shutil.which("usbipd") is not None

def check_wsl():
    return shutil.which("wsl") is not None

def validate_env():
    valid_env = True
    if not check_wsl():
        print("WSL not installed")
        valid_env = False

    if not check_usbipd():
        print("USBIPD not installed")
        valid_env = False

    return valid_env

def process_json_devices(json_list: list[Device]):
    dev_set = set()
    for dev in json_list:
        dev_set.add(Device.from_usbipd_dict(dev))

    return dev_set


def attach_device_to_wsl(dev: Device):
    # print(dev)

    if dev.bus_id is None:
        # print("Bus Id invalid. This sometimes happens with reconnects or as the stlink VHUB is enumerated.")
        return False
    
    print(f"New ST-Link device ({dev.bus_id}). Attaching to WSL")

    if dev.client_ip_address is not None:
        print(f"Bus ({dev.bus_id}) is already attached")
        return False

    output = subprocess.run(WSL_CMD_LITE_WAKEUP, capture_output=True)
    if output.returncode != 0:
        print("Failed to wakeup wsl.")
        return False

    USBIPD_CMD_BIND_BUSID[3] = dev.bus_id
    output = subprocess.run(USBIPD_CMD_BIND_BUSID, capture_output=True)
    if output.returncode == 3:
        print(f"Failed to bind device ({dev.bus_id}). Admin priviledges needed. Re-run via the powershell script or directly as admin.")
        print(output)

        return False

    USBIPD_CMD_ATTACH_BUSID[4] = dev.bus_id
    output = subprocess.run(USBIPD_CMD_ATTACH_BUSID, capture_output=True)
    if output.returncode == 1:
        print(f"Failed to attach to WSL via usbipd. Was the bind successful?")
        print(output)

        return False

    # TODO verify by running lsusb? There doesn't seem to be a correlation between win host bus id
    # and WSL bus id which makes sense.... not sure how to do this. Might need state tracking of some kind
    # output = subprocess.run(WSL_CMD_LSUSB, capture_output=True)
    # print(output)

    return True


def process_new_devices(devices: set[Device]):
    for dev in devices:
        if ST_LINK_DESCR_KEY in dev.description:
            attach_device_to_wsl(dev)


def cleanup_device(dev: Device):
    if dev.bus_id is None:
        # print("Bus Id invalid. This sometimes happens with reconnects or as the stlink VHUB is enumerated.")
        # this also shows as a host disconn when attachment is successful
        return False

    if dev.client_ip_address is not None:
        print(f"Cleaning up ST-Link device ({dev.bus_id}).")

        USBIPD_CMD_DETACH_BUSID[3] = dev.bus_id
        output = subprocess.run(USBIPD_CMD_DETACH_BUSID, capture_output=True)
        if output.returncode == 1:
            # print(f"Failed to detach to WSL via usbipd. The hardware host bus may have disconnected already.")
            # print(output)

            return False


def process_removed_devices(devices: set[Device]):
    for dev in devices:
        if ST_LINK_DESCR_KEY in dev.description:
            cleanup_device(dev)


def shutdown_cleanup():
    if current_state is not None:
        for dev in current_state:
            if dev.bus_id is not None:
                cleanup_device(dev)


def shutdown_handler(sig, frame):
    print("received Ctrl+C shutting down...")
    global running
    running = False


if __name__ == "__main__":
    try:
        valid_env = validate_env()
        if not valid_env:
            print("environment is invalid.")
            exit(1)

        signal.signal(signal.SIGINT, shutdown_handler)

        while running:
            time.sleep(1)

            output = subprocess.run(USBIPD_CMD_STATE, capture_output=True)
            if output.returncode != 0 or output.stderr != b'':
                print("failed to query usb passthrough status via USBIPD STATE.")
                print(output.stderr)

                continue

            latest_state = json.loads(output.stdout)
            latest_state = process_json_devices(latest_state[DEV_KEY])
            if current_state is None:
                process_new_devices(latest_state)
                current_state = latest_state
                continue
            
            added_devices = latest_state - current_state
            removed_devices = current_state - latest_state
            current_state = latest_state

            process_new_devices(added_devices)
            process_removed_devices(removed_devices)

        print("cleaning up lingering devices...")
    except KeyboardInterrupt:
        print("received hard Ctrl+C~ shutting down...")

        running = False
    except Exception as e:
        traceback.print_exc()
        print(e)

        exit(1)

    shutdown_cleanup()

    print("")
    print("")
    print("Daemon shutdown complete.")
    print("")

    exit(0)
    