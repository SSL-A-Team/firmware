"""
Author: Nicholas Witten
Data: 01/18/2025

This script enables smart firmware flashing, allowing the controlling device to
query whether the peripheral device is up-to-date or needs to be flashed.
This script embeds the various firmware image hashes into both the device that
flashes and the device that gets flashed. Since the wheel, dribbler, and kicker
firmware binaries are included inside the control board binary, all of the
firmware hash values can be injected by searching through and modifying only the
control board image. The hash of the various firmware images is taken before the
hash values are injected into the binary, so the stored hash value is zeroed
out for the computation.
"""
import os
import subprocess
import binascii
import traceback
from pathlib import Path


firmware_dir_path = (Path(__file__) / ".." / "..").absolute().resolve()
control_bin_path = firmware_dir_path / "control-board" / "target" / "thumbv7em-none-eabihf" / "release" / "control.bin"
kicker_bin_path = firmware_dir_path / "kicker-board" / "target" / "thumbv7em-none-eabihf" / "release" / "kicker.bin"
wheel_bin_path = firmware_dir_path / "motor-controller" / "build" / "bin" / "wheel.bin"
dribbler_bin_path = firmware_dir_path / "motor-controller" / "build" / "bin" / "dribbler.bin"
wheel_img_hash_magic_ctrl = b'WheelImgHashCtrl'
wheel_img_hash_magic_weel = b'WheelImgHashWeel'
kicker_img_hash_magic_ctrl = b'KickrImgHashCtrl'
kicker_img_hash_magic_kick = b'KickrImgHashKick'
dribbler_img_hash_magic_kick = b'DrblrImgHashKick'
dribbler_img_hash_magic_drbl = b'DrblrImgHashDrbl'
assert len(wheel_img_hash_magic_ctrl) == len(wheel_img_hash_magic_weel) == len(kicker_img_hash_magic_ctrl) == len(kicker_img_hash_magic_kick) == len(dribbler_img_hash_magic_kick) == len(dribbler_img_hash_magic_drbl)
img_hash_offset = len(wheel_img_hash_magic_ctrl)  # Bytes into WheelImgHash struct for the hash value
img_hash_length = 16  # Number of bytes for the hash value


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

def try_embed_img_hash(embed_img_path, img_path, img_hash_magic):
    """
    embed_img_path: path to firmware image that gets the hash injected into
    img_path: path to firmware image to compute hash from
    img_hash_magic: magic bytes that are present in the image at 'embed_img_path' to find hash struct
    """
    try:
        if not os.path.exists(embed_img_path):
            raise Exception(f"embed_img_path - path not found '{embed_img_path}'")
        if not os.path.exists(img_path):
            raise Exception(f"img_path - path not found '{img_path}'")
        img_hash = get_img_hash(img_path)
        embed_img_hash(embed_img_path, img_hash_magic, img_hash)
        print(f"embed_git_status.py - SUCCESS - Embedded hash of '{img_path}' into '{embed_img_path}'")
    except:
        print(f"embed_git_status.py - WARNING - Unable to embed hash of '{img_path}' into '{embed_img_path}'")
        print(traceback.format_exc())


if __name__ == "__main__":
    try_embed_img_hash(control_bin_path, wheel_bin_path, wheel_img_hash_magic_ctrl)
    try_embed_img_hash(control_bin_path, wheel_bin_path, wheel_img_hash_magic_weel)
    try_embed_img_hash(control_bin_path, kicker_bin_path, kicker_img_hash_magic_ctrl)
    try_embed_img_hash(control_bin_path, kicker_bin_path, kicker_img_hash_magic_kick)
    try_embed_img_hash(control_bin_path, dribbler_bin_path, dribbler_img_hash_magic_kick)
    try_embed_img_hash(control_bin_path, dribbler_bin_path, dribbler_img_hash_magic_drbl)