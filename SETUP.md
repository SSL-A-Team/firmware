# Setup

This document will guide you through the first time environment setup of the firmware repository. We
currently support:

| Operating System | Architecture         | Method                     |
|------------------|----------------------|----------------------------|
| Ubuntu 20.04 LTS | x64, aarch64         | Native                     |
| Ubuntu 22.04 LTS | x64, aarch64         | Native                     |
| MacOS 12 Darwin  | x64, M1/M2 (aarch64) | Native                     |
| MacOS 13 Darwin  | x64, M1/M2 (aarch64) | Native                     |
| Windows 11       | x64                  | WSL2 (Ubuntu 20.04, 22.04) |

If you are using Windows, this guide assumes you have already [setup WSL2](https://learn.microsoft.com/en-us/windows/wsl/install) and can enter the Ubuntu shell. 

## Clone the Firmware Repository

Clone the firmware repository into your system.

`git clone https://github.com/SSL-A-Team/firmware.git`


If you are a member of the A-Team:
`git submodule update --init --recursive`


If you are not a member of the A-Team (you will not be able clone our Wifi credentials):
`git -c submodule."robot-private-data".update=none submodule update --init --recursive`
`export NO_ATEAM_WIFI_CREDENTIALS=true`

For Windows users, you must clone the repository from within the WSL2 shell into a WSL2 directory. Do
not use filesystem passthrough. The build tools expect a Linux/Unix environment and Windows paths break
some compilation tools.

## Environment Setup

We use the [Nix package manager](https://nixos.org/) to create reproducible build tools across platforms.
You will need to install Nix for you platform. We provide a wrapper script for all supported platforms,
or you can follow the manual install instructions.

### Automatic Install

The automatic install script will automatically detect the operating system platform and invoke the correct
Nix install command.

 - On Linux the script will install udev rules files.
 - On WSL2 the script will install udev rules files and setup USB passthrough.

`./util/setup.bash`

### Manual Install

If you do not want to use the automated installer we wrote, you may install manually.

Follow the [install instructions](https://nixos.org/download.html) for your platform.

Enable flakes by adding the following to `/etc/nix/nix.conf`, creating the file if needed.
```
experimental-features = nix-command flakes
```

#### Manual Install - Ubnuntu/WSL2 Udev Rules

Run these steps if you are using a manual install and you are on the Ubuntu or WSL2 platforms.

```
cp ./util/udev/49-stlinkv2-1.rules /etc/udev/rules.d/49-stlinkv2-1.rules
cp ./util/udev/49-stlinkv2.rules /etc/udev/rules.d/49-stlinkv2.rules
cp ./util/udev/49-stlinkv3.rules /etc/udev/rules.d/49-stlinkv3.rules
chown root:root /etc/udev/rules.d/49-stlinkv2-1.rules
chown root:root /etc/udev/rules.d/49-stlinkv2.rules
chown root:root /etc/udev/rules.d/49-stlinkv3.rules
chmod 644 /etc/udev/rules.d/49-stlinkv2-1.rules
chmod 644 /etc/udev/rules.d/49-stlinkv2.rules
chmod 644 /etc/udev/rules.d/49-stlinkv3.rules
udevadm control --reload
udevadm trigger
```

#### Manual Install - WSL2 USB Passthrough

You will need to enable USB passthrough to program the robot from WSL2 shell.

```
apt update
apt install linux-tools-generic hwdata
update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20
```

TODO: automate `usbipd wsl attach --busid <busid>`

Additional information is available at [Microsoft USB passthrough documentation](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).
