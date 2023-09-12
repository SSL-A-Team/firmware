#!/usr/bin/env bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PRIVATE_CREDENTIALS_DIR=$SCRIPT_DIR/../credentials/src/private_credentials

if [ ! -d $PRIVATE_CREDENTIALS_DIR ]; then
    git clone git@github.com:SSL-A-Team/ateam-private-credentials.git $PRIVATE_CREDENTIALS_DIR
else
    echo "Private credentials appear to be initialized."
fi