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
import hashlib
from pathlib import Path
from argparse import ArgumentParser


firmware_dir_path = (Path(__file__) / ".." / "..").resolve()
kicker_bin_path = firmware_dir_path / "kicker-board" / "target" / "thumbv7em-none-eabihf" / "release" / "kicker.bin"
wheel_bin_path = firmware_dir_path / "motor-controller" / "build" / "bin" / "wheel-torque.bin"
dribbler_bin_path = firmware_dir_path / "motor-controller" / "build" / "bin" / "dribbler-torque.bin"
wheel_img_hash_magic_ctrl = b'WheelImgHashCtrl'
wheel_img_hash_magic_weel = b'WheelImgHashWeel'
kicker_img_hash_magic_ctrl = b'KickrImgHashCtrl'
kicker_img_hash_magic_kick = b'KickrImgHashKick'
dribbler_img_hash_magic_kick = b'DrblrImgHashKick'
dribbler_img_hash_magic_drbl = b'DrbTrqImgHashDrb'
assert len(wheel_img_hash_magic_ctrl) == len(wheel_img_hash_magic_weel) == len(kicker_img_hash_magic_ctrl) == len(kicker_img_hash_magic_kick) == len(dribbler_img_hash_magic_kick) == len(dribbler_img_hash_magic_drbl)
img_hash_offset = len(wheel_img_hash_magic_ctrl)  # Bytes into ImgHash struct for the hash value
img_hash_length = 16  # Number of bytes for the hash value

# Self-referential magics: the hash of binary X is stored inside binary X itself.
# When computing the hash of X, zero these fields first so the result is stable
# across builds regardless of what was previously embedded.
self_ref_magics = {
    kicker_bin_path:   [kicker_img_hash_magic_kick],
    dribbler_bin_path: [dribbler_img_hash_magic_drbl],
    wheel_bin_path:    [wheel_img_hash_magic_weel],
}


def get_img_hash_stable(fpath):
    """
    Compute md5 of a firmware image with any self-referential hash fields zeroed,
    so the result is stable regardless of what was previously embedded.
    """
    with open(fpath, "rb") as f:
        image = bytearray(f.read())

    for magic in self_ref_magics.get(fpath, []):
        idx = 0
        while True:
            found = image.find(magic, idx)
            if found == -1:
                break
            image[found + img_hash_offset : found + img_hash_offset + img_hash_length] = b'\x00' * img_hash_length
            idx = found + 1

    return hashlib.md5(bytes(image)).digest()

def embed_img_hash(fpath, img_hash_magic, img_hash):
    with open(fpath, "rb") as f:
        image = f.read()

    # Find the firmware hash struct in the image
    struct_start_idx = image.find(img_hash_magic)
    if struct_start_idx == -1:
        raise Exception(f"Unable to find firmware hash magic bytes in {fpath}")
    if image.find(img_hash_magic[:], struct_start_idx+1) != -1:
        raise Exception("Unable to determine firmware hash structure location, magic bytes occur more than once")

    # Construct the new image with an edited firmware hash struct
    new_image = bytearray(image)
    new_image[struct_start_idx+img_hash_offset:struct_start_idx+img_hash_offset+img_hash_length] = img_hash

    # Write the new image replacing the old one
    with open(fpath, "wb") as f:
        f.write(new_image)

def try_embed_img_hash(embed_img_path, img_path, img_hash_magic, img_hash):
    """
    embed_img_path: path to firmware image that gets the hash injected into
    img_path: path to firmware image the hash was computed from (for reporting)
    img_hash_magic: magic bytes present in embed_img_path to find hash struct
    img_hash: precomputed stable hash bytes to embed
    """
    try:
        if not os.path.exists(embed_img_path):
            raise Exception(f"embed_img_path - path not found '{embed_img_path}'")
        embed_img_hash(embed_img_path, img_hash_magic, img_hash)
    except Exception:
        return False
    return True


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--bin", required=True, help="Path to the target binary to embed image hashes in")
    args = parser.parse_args()
    target_bin = Path(args.bin)
    if not target_bin.exists():
        print(f"embed_git_status.py - ERROR - Could not find target binary at path '{target_bin}'")
        exit(1)

    # Precompute stable hashes (self-referential fields zeroed before hashing)
    hashes = {}
    for bin_path in [wheel_bin_path, kicker_bin_path, dribbler_bin_path]:
        if os.path.exists(bin_path):
            hashes[bin_path] = get_img_hash_stable(bin_path)

    success_files = set()
    failure_files = set()
    for source_bin, magic_bytes in [
        (wheel_bin_path,    wheel_img_hash_magic_ctrl),
        (wheel_bin_path,    wheel_img_hash_magic_weel),
        (kicker_bin_path,   kicker_img_hash_magic_ctrl),
        (kicker_bin_path,   kicker_img_hash_magic_kick),
        (dribbler_bin_path, dribbler_img_hash_magic_kick),
        (dribbler_bin_path, dribbler_img_hash_magic_drbl),
    ]:
        if source_bin not in hashes:
            failure_files.add(source_bin)
            continue
        result = try_embed_img_hash(target_bin, source_bin, magic_bytes, hashes[source_bin])
        if result:
            success_files.add(source_bin)
        else:
            failure_files.add(source_bin)
            if source_bin in success_files:
                success_files.remove(source_bin)
    if len(success_files) > 0:
        print(f"embed_git_status.py - SUCCESS - Embedded hash of binary files {', '.join("'" + file.name + "'" for file in success_files)} in binary file '{target_bin.name}'")
    if len(failure_files) > 0:
        print(f"embed_git_status.py - WARNING - Unable to embed hash of binary files {', '.join("'" + file.name + "'" for file in failure_files)} in binary file '{target_bin.name}'. This is expected if the sub-firmware image isn't include in the target binary.")
