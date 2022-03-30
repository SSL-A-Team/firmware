#!/bin/bash

set -e

if grep -q "Ubuntu" /etc/lsb-release; then
	sudo apt-get update
	sudo apt-get install curl
fi

if ! command -v nix > /dev/null; then
	echo "Nix not loaded in the current shell."
	
	if [ ! -d /nix ]; then
		echo "Nix Package Manager not located at /nix."
		echo "Installing..."

		NIX_INSTALLER_NO_MODIFY_PROFILE=true
		sh <(curl -L https://nixos.org/nix/install) --daemon

	else
		echo "Nix appears to be installed."
		echo "Please source nix."
		exit 1
	fi
fi

nix-env -iA nixpkgs.nixFlakes

if [ ! -f ~/.config/nix/nix.conf ]; then
	echo "Creaing nix config file..."
	mkdir -p ~/.config/nix
	touch ~/.config/nix/nix.conf
	echo "Done."
else
	echo "Found existing nix config file."
fi


if ! grep -iq "experimental-features = nix-command flakes" ~/.config/nix/nix.conf; then
	echo "Enabling flakes..."
	echo "experimental-features = nix-command flakes" >> ~/.config/nix/nix.conf
else
	echo "Flakes already enabled."
fi

echo "Setup complete"
exit 0
