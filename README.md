# Firmware

## Environment Setup

We use the (Nix package manager)[https://nixos.org/] to handle dependencies. Install options are below:
- Linux Auto-Setup: `./util/linux_setup.sh`.
- Linux Manual: `$ sh <(curl -L https://nixos.org/nix/install) --daemon`
- OSX Manual: `sh <(curl -L https://nixos.org/nix/install)`

### Enter the Development Environment

Run `nix devleop` to enter a shell with all required dependencies on the path.

Run `exit` to leave the shell.

### Adding Dependencies

Edit flake.nix and run `nix update`. You should commit both flake.nix and flake.lock.

## Building Firmware

Run `make <target>` to build an ELF and flat bin of each target.

## Programming Targets

TODO


