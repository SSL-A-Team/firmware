#!/bin/bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
ANY_FORMAT_CHECK_FAILED=false

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../common/Cargo.toml
if [ $? -ne  0 ]; then
    echo "A-Team common failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../lib-stm32/Cargo.toml
if [ $? -ne  0 ]; then
    echo "A-Team library lib-stm32 failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../lib-stm32-test/Cargo.toml
if [ $? -ne  0 ]; then
    echo "A-Team library test lib-stm32-test failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../credentials/Cargo.toml
if [ $? -ne  0 ]; then
    echo "A-Team library credentials failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../power-board/Cargo.toml
if [ $? -ne  0 ]; then
    echo "A-Team firmware image power-board failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../kicker-board/Cargo.toml
if [ $? -ne  0 ]; then
    echo "A-Team firmware image kicker-board failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

cargo fmt --quiet --check --manifest-path $SCRIPT_DIR/../control-board/Cargo.toml
if [ $? -ne  0 ]; then
    echo "A-Team firmware image control-board failed style check!"
    ANY_FORMAT_CHECK_FAILED=true
fi

echo ""
echo ""

if [ "$ANY_FORMAT_CHECK_FAILED" = true ]; then
    echo "############"
    echo "#  FAILED  #"
    echo "############"
else
    echo "############"
    echo "#  PASSED  #"
    echo "############"
fi

echo ""