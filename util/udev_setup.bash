#!/usr/bin/env bash

# prompt for root
# https://askubuntu.com/questions/746350/request-root-privilege-from-within-a-script
echo "$(whoami)"
[ "$UID" -eq 0 ] || exec sudo "$0" "$@"

set -e

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

# check if we're in WSL, this will complete some extra steps for usb passthrough
if grep -qi microsoft /proc/version; then
    echo "DETECTED PLATFORM - WSL"
    
    echo "WSL - setup USB passthrough"
    
    KERNEL_VERSION=$(uname -r)
    echo "WSL - found kernel version $KERNEL_VERSION"

    echo "WSL - install usbip..."
    apt update
    apt install linux-tools-$KERNEL_VERSION hwdata
    update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/$KERNEL_VERSION/usbip 20
    echo "WSL - platform specific setup complete."
elif grep -q "Ubuntu" /etc/lsb-release; then
    echo "DETECTED PLATFORM - Native Linux"
else
    echo "UNKNOWN PLATFORM. Nothing to do."
    exit 0
fi

UDEV_RULE_FILES_LOCATION=/etc/udev/rules.d

any_rules_file_installed=false
for udev_rule_file in $SCRIPT_DIR/udev/*.rules; do
    udev_rule_file_basename=$(basename $udev_rule_file)
    echo "Found candidate STLink rules file: $udev_rule_file_basename"

    perform_install=false
    rulefile_install_path=$UDEV_RULE_FILES_LOCATION/$udev_rule_file_basename
    if [ ! -f $rulefile_install_path ]; then
        echo -e "\tSTLink udev rule file not installed."
        perform_install=true
    else
        GOLDEN_CHECKSUM=`md5sum $udev_rule_file | cut -d\  -f1`
        EXISTING_CHECKSUM=`md5sum $rulefile_install_path | cut -d\  -f1`

        if [ "$GOLDEN_CHECKSUM" != "$EXISTING_CHECKSUM" ]; then
            echo -e "\tSTLink udev rule checksum failed."
            perform_install=true
        else
            echo -e "\tSTLink udev rule file is already installed."
        fi
    fi

    if [ $perform_install = true ]; then
        echo -e "\tInstalling rules file... ($udev_rule_file -> $rulefile_install_path)"
        # copy the rules file
        cp $udev_rule_file $rulefile_install_path
        # make root the owner
        chown root:root $rulefile_install_path
        # set permission to root RW, othes R (644)
        chmod 644 $rulefile_install_path

        any_rules_file_installed=true
        echo -e "Done."
    fi
done

if [ $any_rules_file_installed = true ]; then
    echo "One or more rules file was installed. Reloading udevadm..."
    udevadm control --reload
    udevadm trigger
    echo "Done."
else
    echo "No rules files were installed. No further actions to complete."
fi