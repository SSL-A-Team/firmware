# Firmware ![example workflow](https://github.com/SSL-A-Team/firmware/actions/workflows/CI.yml/badge.svg)

This is the unified firmware repository for the A-Team.

## Environment Setup

We use the [Nix package manager](https://nixos.org/) to handle dependencies. Install options are below:
- Linux Auto-Setup: `./util/linux_setup.sh`.
- Linux Manual: `sh <(curl -L https://nixos.org/nix/install) --daemon`
- Windows Subsystem for Linux (WSL) Manual: `sh <(curl -L https://nixos.org/nix/install) --no-daemon`
- OSX Manual: `sh <(curl -L https://nixos.org/nix/install)`

Enable flakes by adding the following to `/etc/nix/nix.conf`:
```
experimental-features = nix-command flakes
```

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

## Building Firmware

Run `make <target>` to build an ELF and flat bin of each target. This wraps both CMake C/C++ targets and Rust targets. Suggest tab autocomplete to enumerate all targets.

Most targets have several variants:
 - `<name>` build the target in release mode
 - `<name>-prog` build the target in release mode, write to flash
 - `<name>-debug` build the target in debug mode
 - `<name>-debug-gdb` build the target in debug mode, write to flash, attach gdb under phy target reset
 - `<name>-debug-mon` build the target in debug mode, attach a monitor to the output of `hprintln!` macros
 - `<name>-debug-prog` build the target in debug mode, write to flash


