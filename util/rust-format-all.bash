#!/bin/bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

cargo fmt --manifest-path $SCRIPT_DIR/../common/Cargo.toml
cargo fmt --manifest-path $SCRIPT_DIR/../lib-stm32/Cargo.toml
cargo fmt --manifest-path $SCRIPT_DIR/../lib-stm32-test/Cargo.toml
cargo fmt --manifest-path $SCRIPT_DIR/../credentials/Cargo.toml
cargo fmt --manifest-path $SCRIPT_DIR/../power-board/Cargo.toml
cargo fmt --manifest-path $SCRIPT_DIR/../kicker-board/Cargo.toml
cargo fmt --manifest-path $SCRIPT_DIR/../control-board/Cargo.toml