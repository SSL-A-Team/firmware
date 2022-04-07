#!/bin/bash

set -e

ODIN_FW_CLONE_URL=git@github.com:SSL-A-Team/odin-w26x-binary-private.git
ODIN_FW_GIT_TAG="v8.2.0_1.0"

BASE=$(readlink -f $(dirname $0)/..)
PRIVATE_BINARY_BLOBS_DIR=$BASE/private_binary_blobs

if [ ! -d $PRIVATE_BINARY_BLOBS_DIR ]; then
	mkdir -p $PRIVATE_BINARY_BLOBS_DIR
fi

if [ ! -f $PRIVATE_BINARY_BLOBS_DIR/odin-w26x ]; then
	git clone $ODIN_FW_CLONE_URL $PRIVATE_BINARY_BLOBS_DIR/odin-w26x
fi


pushd $PRIVATE_BINARY_BLOBS_DIR/odin-w26x
git fetch --all --tags
git checkout $ODIN_FW_GIT_TAG
popd
