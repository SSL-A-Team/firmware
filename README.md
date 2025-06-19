# Firmware ![example workflow](https://github.com/SSL-A-Team/firmware/actions/workflows/CI.yml/badge.svg)

This is the unified firmware repository for the A-Team.

## Setup

Follow the [first time setup guide](SETUP.md).

If you are not a member of the A-Team, you will not have access to the private repository that store our Wifi credentials.
Please set `export NO_ATEAM_WIFI_CREDENTIALS=true` in your shell, so the build system will load dummy credentials. You can
set them in `ateam-credentials/src/public_credentials/wifi.rs`.

## Building Firmware

Begin by entering the Nix environment `nix develop`. If you use a shell other than bash, you can try `nix develop -c $SHELL` 
to stay in your preferred shell.

WSL users should launch VS Code from the nix environment in the Ubuntu shell.

A top level Makefile is provided for all targets. Targets in the form `<module>--<binary>--<action>`.

The primary command `make control-board--control--run` will build all robot firmware, flash hardware, and run the image.

### List of Modules

| Module                  | Description                                                                           |
|-------------------------|---------------------------------------------------------------------------------------|
| common                  | common libraries shared across modules                                                |
| software\-communication | packet defintitions used for communication with software and between hardware modules |
| motor-controller        | firmware binaries for stspin32 motor controllers (wheels, dribbler, tests)            |
| kicker-board            | fimrware binaries for the kicker board                                                |
| control-board           | firwmare binaries for the control board                                               |

### List of Binaries

| Binaries                | Description                                                       |
|-------------------------|-------------------------------------------------------------------|
| Varied.                 | See README.md files in subfolders or use tab complete suggestion. |

### List of Actions

| Actions                         | Descrption                                                                                     |
|---------------------------------|------------------------------------------------------------------------------------------------|
| `<module>--all`               | Builds all binaries for the module                                                             |
| `<module>--<binary>`          | Builds the binary for the module                                                               |
| `<module>--<binary>--run`   | Flashes the binary to the hardware target, runs the binary, print messages appear in the shell |
| `<module>--<binary>--debug` | Flashes the binary to the hardware target, attaches gdb, halts the processor at reset_vector   |
