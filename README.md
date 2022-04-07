# Firmware ![example workflow](https://github.com/SSL-A-Team/firmware/actions/workflows/CI.yml/badge.svg)

This is the unified firmware repository for the A-Team.

## Environment Setup

We use the [Nix package manager](https://nixos.org/) to handle dependencies. Install options are below:
- Linux Auto-Setup: `./util/linux_setup.sh`.
- Linux Manual: `sh <(curl -L https://nixos.org/nix/install) --daemon`
- OSX Manual: `sh <(curl -L https://nixos.org/nix/install)`

### Enter the Development Environment

Run `nix develop` to enter a shell with all required dependencies on the path.

Run `exit` to leave the shell.

### Adding Dependencies

Edit flake.nix and run `nix update`. You should commit both flake.nix and flake.lock.

### Supported Platforms

We support the following platforms that have a valid Nix install:
- "aarch64-darwin"
- "aarch64-linux"
- "x86\_64-darwin"
- "x86\_64-linux"

### Binary Blobs
The radio binary blob is *optional* to build our firmware. It is only needed if the default
firmware is clobbered by a mistargeted flash command (yes we learned this the hard way).

Team members can run `./util/binary_blob_setup.sh` to acquire the firmware.

Our 5GHz radio uses a private binary blob provided by the radio manufacturer u-blox. Access can be
requested via a support ticket and is free. Additionally purchased radios come preloaded with firmware.
Sometimes it is necessary to recover or update the firmware, which is why the `binary\_blobs\_setup.sh`
script exists. You will need to update the variables at the top of the script to point to your private
mirror and tag.

The support portal is located [here](https://portal.u-blox.com).

## Building Firmware

Run `make <target>` to build an ELF and flat bin of each target. Available targets:

- all
- clean
- test-breadboard-radio

## Programming Targets

- test-breadboard-radio-prog


