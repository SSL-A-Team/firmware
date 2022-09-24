.PHONY: all

.DEFAULT_GOAL := all

.PHONY: .setup
.setup:
	mkdir -p build/ && \
	cd build && \
	cmake .. \

.PHONY: clean
clean:
	rm -rf build/

objcopy := arm-none-eabi-objcopy
motor_openocd_cfg_file := board/st_nucleo_h743zi.cfg

motor_binaries := ${shell cd motor/src/bin && ls -d * && cd ../../..}

define create-rust-targets
$1-$2: dev3204-wheel
	cd $1/ && \
	cargo build --release --bin $2
all:: $1-$2

$1-$2-prog: $1-$2
	./util/program.sh $3 $1/target/thumbv7em-none-eabihf/release/$2

$1-$2-debug: dev3204-wheel
	cd $1/ && \
	cargo build --bin $2
all:: $1-$2-debug

$1-$2-debug-prog:
	./util/program.sh $3 $1/target/thumbv7em-none-eabihf/debug/$2

$1-$2-debug-mon:
	./util/program.sh $3 $1/target/thumbv7em-none-eabihf/debug/$2 monitor

$1-$2-debug-gdb:
	./util/attach_gdb.sh $3 $1/target/thumbv7em-none-eabihf/debug/$2

endef
$(foreach element,$(motor_binaries),$(eval $(call create-rust-targets,motor,$(element),$(motor_openocd_cfg_file))))

stspin_binaries := ${shell cd stspin/bin && ls -d * && cd ../..}
define create-cmake-targets
$1: .setup
	cd build/ && \
	make $1
all:: $1

$1-prog: $1
	cd build/ && \
	make $1-prog

$1-gdb: $1
	cd build/ && \
	make $1-gdb
endef
$(foreach element,$(stspin_binaries),$(eval $(call create-cmake-targets,$(element))))
