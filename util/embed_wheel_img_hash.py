"""
Author: Nicholas Witten
Data: 01/18/2025

Embeds the wheel image hash into both the control board and wheel firmware
binaries. Since the wheel firmware binary is included inside the control board
binary, this can be done by searching through and modifying only the control
board image. The hash of the wheel image changes after inserting the hash value
itself, so the value of the pre-inserted image hash (the value is zeroed out) is
used on both the control board and wheel sides.
"""
import os
import subprocess
import binascii
import traceback


firmware_dir_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
control_bin_path = os.path.join(firmware_dir_path, "control-board/target/thumbv7em-none-eabihf/release/control")
wheel_img_path = os.path.join(firmware_dir_path, "motor-controller/build/bin/wheel.bin")
wheel_img_hash_magic_control_board = b'WheelImgHashCtrl'
wheel_img_hash_magic_motor_controller = b'WheelImgHashMotr'
assert len(wheel_img_hash_magic_control_board) == len(wheel_img_hash_magic_motor_controller)
img_hash_offset = len(wheel_img_hash_magic_control_board)  # Bytes into WheelImgHash struct
img_hash_length = 16


def get_img_hash(fpath):
    p = subprocess.run(["md5sum", fpath], text=True, stdout=subprocess.PIPE)
    if p.returncode != 0:
        raise Exception(f"Unable to get hash, command failed: md5sum {fpath}")
    hash_string = p.stdout.split(" ")[0]
    return binascii.unhexlify(hash_string)  # turn into bytes object

def embed_img_hash(fpath, img_hash_magic, img_hash):
    with open(fpath, "rb") as f:
        image = f.read()

    # Find the firmware hash struct in the image
    struct_start_idx = image.find(img_hash_magic)  # It'll be backwards because of little endian byte
    if struct_start_idx == -1:
        raise Exception(f"Unable to find firmware hash magic bytes in {fpath}")
    # is it possible this sequence happens to occur more than once?
    if image.find(img_hash_magic[:], struct_start_idx+1) != -1:
        raise Exception("Unable to determine firmware hash structure location, magic bytes occur more than once")

    # Construct the new image with an edited firmware hash struct
    new_image = bytearray(image)
    new_image[struct_start_idx+img_hash_offset:struct_start_idx+img_hash_offset+img_hash_length] = img_hash

    # Write the new image replacing the old one
    with open(fpath, "wb") as f:
        f.write(new_image)

def try_embed_wheel_img_hash(fpath):
    try:
        if not os.path.exists(fpath):
            raise Exception(f"path not found - {fpath}")
        if not os.path.exists(wheel_img_path):
            raise Exception(f"path not found - {wheel_img_path}")
        img_hash = get_img_hash(wheel_img_path)
        embed_img_hash(fpath, wheel_img_hash_magic_control_board, img_hash)
        embed_img_hash(fpath, wheel_img_hash_magic_motor_controller, img_hash)
        print(f"embed_git_status.py - SUCCESS - Embedded motor img hash into {fpath}")
    except:
        print(f"embed_git_status.py - WARNING - Unable to embed the firmware hash in {fpath}")
        print(traceback.format_exc())


if __name__ == "__main__":
    img_path = os.path.join(firmware_dir_path, f"control-board/target/thumbv7em-none-eabihf/release/control")
    try_embed_wheel_img_hash(img_path)
    # Do we use control.bin for anything?
    img_path = img_path + ".bin"
    try_embed_wheel_img_hash(img_path)