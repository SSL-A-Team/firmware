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

Edit flake.nix and run `nix flake update`. You should commit both flake.nix and flake.lock.

### Supported Platforms

We support the following platforms that have a valid Nix install:
- "aarch64-darwin"
- "aarch64-linux"
- "x86\_64-darwin"
- "x86\_64-linux"

### Configuring VS Code Rust Analyzer 

By default the rust-analyzer targets the host system, which is wrong since our targets are cross compiled. The following snippet
correctly sets the target. You'll need to run vscode from the `nix develop` shell so it can find the correct cross target
analyser.

```
{
    "rust-analyzer.cargo.target": "thumbv7em-none-eabihf",
    "rust-analyzer.checkOnSave.allTargets": false,
    "rust-analyzer.server.path": "/nix/store/gghbkcpaqzrs0hysswi88s1l4slc8cbd-rust-analyzer-2022-06-13/bin/rust-analyzer",
    "rust-analyzer.procMacro.server": null,
}
```

This should probably somehow be included in the repo directly in a settings.json file but I'm haven't looked up how to do it.

## Building Firmware

Run `make <target>` to build an ELF and flat bin of each target. This wraps both CMake C/C++ targets and Rust targets. Suggest tab autocomplete to enumerate all targets.

Most targets have several variants:
 - `<name>` build the target in release mode
 - `<name>-prog` build the target in release mode, write to flash
 - `<name>-debug` build the target in debug mode
 - `<name>-debug-gdb` build the target in debug mode, write to flash, attach gdb under phy target reset
 - `<name>-debug-mon` build the target in debug mode, attach a monitor to the output of `hprintln!` macros
 - `<name>-debug-prog` build the target in debug mode, write to flash

## Passing Through a USB Device

[Microsoft USB Passthrough Documentation](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)

Above link for the passthrough on Win11/WSL2 works fine except for one tweak:
```
sudo apt install linux-tools-5.4.0-77-generic hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/5.4.0-77-generic/usbip 20```
```

should be

```
sudo apt install linux-tools-5.15.0-52-generic hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/5.15.0-52-generic/usbip 20
```

TODO: script kernel version so this always work

After passing the device through you need to edit the permissions for the device in your udev rules such that a user who is not root can read/write for the tests to work.

In order to do this you need to add a file named `50-usb-stm32.rules` to `etc/udev/rules.d` that contains the following line:
`SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="0666"`
Then run the command `sudo udevadm test $(udevadm info -q path -n /dev/bus/usb/<bus #>/<device #>)`. You can find the bus # and the device # by running `lsusb` in your wsl window.
You may need to restart your computer or relog into wsl for the tests to begin working. (Restarting will require you  to re-pass the usb)
