#!/bin/sh -e

export DEBIAN_FRONTEND=noninteractive

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

apt update
apt install -y \
  bzip2 \
  ca-certificates \
  curl \
  locales \
  sudo \
  xz-utils

localedef -f UTF-8 -i en_US -A /usr/share/locale/locale.alias -c en_US.UTF-8

cp $SCRIPT_DIR/nix.conf /tmp

groupadd -g 30000 --system nixbld

for i in $(seq 1 32); do
  useradd \
    --home-dir /var/empty \
    --gid 30000 \
    --groups nixbld \
    --no-user-group \
    --system \
    --shell /usr/sbin/nologin \
    --uid $((30000 + i)) \
    --password "!" \
    nixbld$i
done

mkdir -p \
  /root/.config/nix \
  /root/.nixpkgs
mv /tmp/nix.conf /root/.config/nix/nix.conf
echo "{ allowUnfree = true; }" > /root/.nixpkgs/config.nix

cd /tmp
curl https://nixos.org/releases/nix/nix-2.3/nix-2.3-x86_64-linux.tar.xz | tar xJf -
cd nix-2.3-x86_64-linux
USER=root ./install --no-daemon

export NIX_PATH=nixpkgs=/root/.nix-defexpr/channels/nixpkgs:/root/.nix-defexpr/channels
export NIX_SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt
export PATH=/root/.nix-profile/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export SUDO_FORCE_REMOVE=yes

nix-channel --update
nix-env -iA \
  nixpkgs.nix

rm -rf /var/lib/apt/lists/*
nix-channel --remove nixpkgs
rm -rf /nix/store/*-nixpkgs*
nix-collect-garbage -d
nix-store --verify --check-contents
nix optimise-store
rm -rf /tmp/*
