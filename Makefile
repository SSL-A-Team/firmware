.PHONY: all
.DEFAULT_GOAL := all

.PHONY: .cmake-setup
.cmake-setup:
	mkdir -p build/ && \
	cd build && \
	cmake .. \

clean-stspin:
	rm -rf build

clean-motor: clean-stspin
	cd motor-embassy && \
	cargo clean

.PHONY: clean
clean: clean-stspin clean-motor software-communication-clean

.PHONY: test

############################
#  Software Communication  #
############################

software-communication-clean:
	cd software-communication && \
	make clean

software-communication-build:
	cd software-communication && \
	make all
all:: software-communication-build

software-communication-test:
	cd software-communication && \
	make test
test:: software-communication-test

#####################
#  STSPIN binaries  #
#####################

.PHONY: stspin-all
stspin_binaries := ${shell cd stspin/bin && ls -d * && cd ../..}

define create-cmake-targets
$1: .cmake-setup
	cd build/ && \
	make $1
all:: $1
stspin-all:: $1

$1-prog: $1
	cd build/ && \
	make $1-prog

$1-gdb: $1
	cd build/ && \
	make $1-gdb
endef
$(foreach element,$(stspin_binaries),$(eval $(call create-cmake-targets,$(element))))

#########################
#  Motorboard Binaries  #
#########################

.PHONY: motorboard-all
motor_binaries := ${shell cd motor-embassy/src/bin && ls -d * && cd ../../..}
motor_openocd_cfg_file := board/st_nucleo_h743zi.cfg

define create-rust-targets
$1-$2: stspin-all
	cd $1/ && \
	cargo build --release --bin $2
motorboard-all:: $1-$2

$1-$2-run: stspin-all
	cd $1/ && \
	cargo run --release --bin $2

$1-$2-prog: $1-$2
	./util/program.sh $3 $1/target/thumbv7em-none-eabihf/release/$2

$1-$2-debug: stspin-all
	cd $1/ && \
	cargo build --bin $2
motorboard-all:: $1-$2-debug

$1-$2-debug-run: stspin-all
	cd $1/ && \
	cargo run --bin $2

$1-$2-debug-prog: $1-$2-debug
	./util/program.sh $3 $1/target/thumbv7em-none-eabihf/debug/$2

endef
$(foreach element,$(motor_binaries),$(eval $(call create-rust-targets,motor-embassy,$(element),$(motor_openocd_cfg_file))))

all:: stspin-all motorboard-all
