#!/bin/sh -e

export DEBIAN_FRONTEND=noninteractive

echo "install packages..."

apt update
apt install -y \
  bzip2 \
  ca-certificates \
  curl \
  locales \
  sudo \
  xz-utils

echo "setup locale..."

localedef -f UTF-8 -i en_US -A /usr/share/locale/locale.alias -c en_US.UTF-8

echo "setup nix groups..."

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

echo "configure nix install..."

mkdir -p \
  /root/.config/nix \
  /root/.nixpkgs
mv /tmp/nix.conf /root/.config/nix/nix.conf
echo "{ allowUnfree = true; }" > /root/.nixpkgs/config.nix

echo "install nix..."

cd /tmp
curl https://releases.nixos.org/nix/nix-2.6.1/nix-2.6.1-x86_64-linux.tar.xz
tar xJf ./nix-2.6.1-x86_64-linux.tar.xz
cd nix-2.6.1-x86_64-linux
USER=root ./install --no-daemon

echo "setup nix paths..."

export NIX_PATH=nixpkgs=/root/.nix-defexpr/channels/nixpkgs:/root/.nix-defexpr/channels
export NIX_SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt
export PATH=/root/.nix-profile/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export SUDO_FORCE_REMOVE=yes

echo "udpate nix package manifest..."

nix-channel --update
nix-env -iA \
  nixpkgs.nix

echo "cleanup..."

rm -rf /var/lib/apt/lists/*
nix-channel --remove nixpkgs
rm -rf /nix/store/*-nixpkgs*
nix-collect-garbage -d
nix-store --verify --check-contents
nix optimise-store
rm -rf /tmp/*

echo "setup complete..."
