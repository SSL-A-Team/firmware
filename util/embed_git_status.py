"""
Author: Nicholas Witten
Data: 01/18/2025

Embeds the git hash and git dirty status into the motor-controller and control-board firmware binaries
"""
import os
import subprocess
import binascii
import argparse
import traceback


firmware_dir_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
git_id_sequence_control_board = b'\x67\x89\xAB\xCD'
git_id_sequence_motor_controller = b'\xAA\xBB\xCC\xDD'
git_hash_offset = 4 # bytes into the GitStatus_t struct
git_dirty_offset = 8  # bytes into the GitStatus_t struct


def embed_git_status(fpath, git_struct_id):
    with open(fpath, "rb") as f:
        binary = f.read()

    # Find the git status struct in the binary
    sequence_start_idx = binary.find(git_struct_id[::-1])  # It'll be backwards because of little endian byte
    if sequence_start_idx == -1:
        raise Exception(f"Unable to find git status structure in {fpath}")
    # is it possible this sequence happens to occur more than once?
    if binary.find(git_struct_id[::-1], sequence_start_idx+1) != -1:
        raise Exception("Unable to determine git status structure location, ID sequence occurs more than once")

    # Run git command to get the current commit hash
    result = subprocess.run(['git', 'rev-parse', 'HEAD'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if result.returncode == 0:
        git_hash = result.stdout.strip()
    else:
        raise Exception("Failed to get git hash of HEAD")

    # Run git command to check if the tree is dirty
    result = subprocess.run(['git', 'status', '--short'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if result.returncode == 0:
        if len(result.stdout.strip()) == 0:
            git_dirty = False
        else:
            git_dirty = True
    else:
        raise Exception("Failed to run git status")

    # Construct the values for git_status and git_hash to embed in the binary
    git_hash_value = binascii.unhexlify(git_hash[:8])[::-1]  # first 8 characters of the hash string, first 4 bytes of hash, backwards for little endian
    if git_dirty:
        git_dirty_value = b'\x00\x00\x00\x01'[::-1]  # backwards for little endian
    else:
        git_dirty_value = b'\x00\x00\x00\x00'[::-1]  # backwards for little endian

    # Construct the new binary with an edited git status struct
    new_binary = bytearray(binary)
    new_binary[sequence_start_idx+git_dirty_offset:sequence_start_idx+git_dirty_offset+4] = git_dirty_value
    new_binary[sequence_start_idx+git_hash_offset:sequence_start_idx+git_hash_offset+4] = git_hash_value

    # Write the new binary replacing the old one
    with open(fpath, "wb") as f:
        f.write(new_binary)

def try_embed_git_status(fpath, git_struct_id):
    try:
        if os.path.exists(fpath):
            embed_git_status(fpath, git_struct_id)
            print(f"embed_git_status.py - SUCCESS - Embedded the current git tree status into {fpath}")
        else:
            error_message = f"path not found - {fpath}"
            print(error_message)
            raise Exception(error_message)
    except:
        print(f"embed_git_status.py - WARNING - Unable to embed the current git tree status into {fpath}")
        print(traceback.format_exc())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Embeds the git hash and git dirty status into the motor-controller firmware binaries")
    parser.add_argument('-t', '--target', type=str, help="Name of the target: motor-controller--wheel, motor-controller--dribbler, control-board--control)", required=True)
    args = parser.parse_args()

    target_module = args.target.split("--")[0]
    target_name = args.target.split("--")[1]

    if target_module == "motor-controller":
        bin_path = os.path.join(firmware_dir_path, f"motor-controller/build/bin/{target_name}.bin")
        git_id_sequence = git_id_sequence_motor_controller
        try_embed_git_status(bin_path, git_id_sequence)
    elif target_module == "control-board":
        git_id_sequence = git_id_sequence_control_board
        bin_path = os.path.join(firmware_dir_path, f"control-board/target/thumbv7em-none-eabihf/release/{target_name}")
        try_embed_git_status(bin_path, git_id_sequence)
        # Do we use control.bin for anything?
        bin_path = bin_path + ".bin"
        try_embed_git_status(bin_path, git_id_sequence)
    else:
        raise Exception(f"Unrecognized target '{args.target}'")

    # Print out the new bytes
    # with open(bin_path, "rb") as f:
    #     new_binary = f.read()
    # sequence_start_idx = new_binary.find(git_id_sequence[::-1])
    # print(' '.join(f'{byte:02x}' for byte in new_binary[sequence_start_idx:sequence_start_idx+12]))