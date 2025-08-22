#!/bin/bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
ANY_FORMAT_CHECK_FAILED=false

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../common/Cargo.toml
if [ $? -ne  0]; then
    echo "A-Team Common failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../lib-stm32/Cargo.toml
if [ $? -ne  0]; then
    echo "A-Team Common failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../lib-stm32-test/Cargo.toml
if [ $? -ne  0]; then
    echo "A-Team Common failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../credentials/Cargo.toml
if [ $? -ne  0]; then
    echo "A-Team Common failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../power-board/Cargo.toml
if [ $? -ne  0]; then
    echo "A-Team Common failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../kicker-board/Cargo.toml
if [ $? -ne  0]; then
    echo "A-Team Common failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../control-board/Cargo.toml
if [ $? -ne  0]; then
    echo "A-Team Common failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

echo ""
echo ""

if [ "$ANY_FORMAT_CHECK_FAILED" = true ]; then
    echo "One or more file failed the style check!"
elif
    echo "All files passed the style check."
fi

echo ""