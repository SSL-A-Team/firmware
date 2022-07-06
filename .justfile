default: motor stspin

_setup:
    mkdir -p build && \
    cd build && \
    cmake ..

# clean stuff
clean:
    rm -rf bin
    rm -rf build

_cargo DIR BIN CMD:
    #!/bin/sh
    [ "{{DIR}}" == "{{BIN}}" ] && bin_name="{{BIN}}" || bin_name="{{DIR}}-{{BIN}}"
    echo $bin_name
    cd {{DIR}}
    if [ '{{CMD}}' == 'build' ]; then
        cargo build --release --bin {{BIN}} && \
        mkdir -p ../bin && \
        arm-none-eabi-objcopy -Obinary target/*/release/{{BIN}} ../bin/$bin_name.bin
    elif [ '{{CMD}}' == 'prog' ]; then
        cargo run --release --bin {{BIN}}
    elif [ '{{CMD}}' == 'monitor' ]; then
        cargo run --release --bin {{BIN}} monitor
    else
    cat << EOF
    Error: Invalid command '{{CMD}}'

    Valid commands are:
        build   - runs build
        prog    - builds and programs
        monitor - builds, programs, then leaves openocd open
    EOF
    fi

stspin CMD='build':     (_cargo 'stspin' 'stspin' CMD)
stspin-uart-ping CMD='build': (_cargo 'stspin' 'uart-ping' CMD)

motor CMD='build': (_cargo 'motor' 'motor' CMD)
motor-bootloader CMD='build': stspin (_cargo 'motor' 'bootloader' CMD)
motor-nucleo-blinky CMD='build': (_cargo 'motor' 'nucleo-blinky' CMD)
