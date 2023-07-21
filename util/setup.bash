#!/bin/bash

set -e

PLATFORM="none"
if grep -qi microsoft /proc/version; then
	echo "DETECTED PLATFORM - WSL"
	PLATFORM="WSL"
elif grep -q "Ubuntu" /etc/lsb-release; then
    echo "DETECTED PLATFORM - Native Linux"
	PLATFORM="Ubuntu"
elif [[ $OSTYPE == 'darwin'* ]]; then
	echo "DETECTED PLATFORM - MacOS"
	PLATFORM="MacOS"
else
	echo "UNKNOWN PLATFORM"
	exit 1
fi

if [[ $PLATFORM == 'WSL' ]] || [[ $PLATFORM == "Ubuntu" ]]; then
	if ! command -v curl > /dev/null; then
		sudo apt update
		sudo apt install -y curl
	fi
fi

if ! command -v nix > /dev/null; then
	echo "Nix not loaded in the current shell."
	
	if [ ! -d /nix ]; then
		echo "Nix Package Manager not located at /nix."
		echo "Installing..."

		# there seem to be some issues with auto profile not working on MacOS, turn it off
		if [[ $PLATFORM == "MacOS" ]]; then
			NIX_INSTALLER_NO_MODIFY_PROFILE=true
		fi

		if [[ $PLATFORM == "Ubuntu" ]] || [[ $PLATFORM == "MacOS" ]]; then
			sh <(curl -L https://nixos.org/nix/install) --daemon
		elif [[ $PLATFORM == "WSL" ]]; then
			sh <(curl -L https://nixos.org/nix/install) --no-daemon
		fi
	fi

	# nix should be installed now (either previous installed and not sourced, or newly installed)
	# look for it and source it
	if [ -e "~/.nix-profile/etc/profile.d/nix.sh" ]; then
		echo "Found Nix installed as single-user mode."
		. '~/.nix-profile/etc/profile.d/nix.sh'
		echo "Loaded Nix to current shell."
	elif [ -e '/nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh' ]; then
		echo "Found Nix installed as multi-user mode."
		. '/nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh'
		echo "Loaded Nix to current shell."
	else
		echo "Some nix elements exist, but the entry points could not be found."
	fi

	if ! command -v nix > /dev/null; then
		echo "Nix not loaded in the current shell after efforts to find and load it."
		echo "Unclear on how to proceed. Please install Nix and source it to the shell before trying again."
		exit 1
	fi
fi

nix-env -iA nixpkgs.nixFlakes

# check for or create a Nix config file
if [ ! -f ~/.config/nix/nix.conf ]; then
	echo "Creaing nix config file..."
	mkdir -p ~/.config/nix
	touch ~/.config/nix/nix.conf
	echo "Done."
else
	echo "Found existing nix config file."
fi

# check for and enable flakes
if ! grep -iq "experimental-features = nix-command flakes" ~/.config/nix/nix.conf; then
	echo "Enabling flakes..."
	echo "experimental-features = nix-command flakes" >> ~/.config/nix/nix.conf
else
	echo "Flakes already enabled."
fi

# setup programming adapters
if [[ $PLATFORM == 'WSL' ]] || [[ $PLATFORM == "Ubuntu" ]]; then
	echo "calling script to setup STLink/JLink programmers..."
	SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
	$SCRIPT_DIR/udev_setup.bash
fi

echo "Setup complete"
exit 0
