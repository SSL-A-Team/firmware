.PHONY: test

ifdef NO_ATEAM_WIFI_CREDENTIALS
ifeq ($(NO_ATEAM_WIFI_CREDENTIALS),true)
$(warning "Building without A-Team Wifi credentials.")
additional_control_cargo_flags := --features=no-private-credentials
else
$(info "Building with A-Team Wifi credentials.")
additional_control_cargo_flags :=
endif
else
$(info "Building with A-Team Wifi credentials.")
additional_control_cargo_flags :=
endif

############################
#  software communication  #
############################

software-communication--clean:
	cd software-communication/ && \
	make clean

software-communication--build:
	cd software-communication/ && \
	make all
all:: software-communication--build

software-communication--test:
	cd software-communication/ && \
	make test
test:: software-communication--test

####################
#  common targets  #
####################

.PHONY: common--all
common_binaries := ${shell cd common/src/bin && ls -d * && cd ../../..}

define create-common-targets
.PHONY: .$1-$2-cargo-build
.$1-$2-cargo-build:
	cd $1/ && \
	cargo build --release --bin $2

.PHONY: .$1-$2-cargo-build-debug
.$1-$2-cargo-build-debug:
	cd $1/ && \
	cargo build --bin $2

$1--$2: .$1-$2-cargo-build
$1--all:: $1--$2

$1--$2--run: $1--$2
	cd $1/ && \
	cargo run --release --bin $2

$1--$2--debug: .$1-$2-cargo-build-debug
$1--all:: $1--$2--debug

$1--$2--debug-run: $1--$2--debug
	cd $1/ && \
	cargo run --bin $2
endef
$(foreach element,$(common_binaries),$(eval $(call create-common-targets,common,$(element))))

common--test:
	cd common/ && \
	cargo test
test:: common--test

common--clean:
	cd common/ && \
	cargo clean

###############################
#  motor controller binaries  #
###############################

 motor-controller/build/CMakeCache.txt: motor-controller/CMakeLists.txt
	cd motor-controller/ && \
	cmake -B build/ \

.PHONY: motor-controller--all
motor_controller_binaries := ${shell cd motor-controller/bin && ls -d * && cd ../..}

define create-motor-controller-targets
$1--$2: motor-controller/build/CMakeCache.txt
	cd $1/build/ && \
	make $2
$1--all:: $1--$2

$1--$2--prog: $1--$2
	cd $1/build/ && \
	make $2-prog

$1--$2--debug: $1--$2
	cd $1/build/ && \
	make $2-gdb
endef
$(foreach element,$(motor_controller_binaries),$(eval $(call create-motor-controller-targets,motor-controller,$(element))))

motor-controller--clean:
	rm -rf motor-controller/build

#####################
#  kicker binaries  #
#####################

.PHONY: kicker-board--all
kicker_binaries := ${shell cd kicker-board/src/bin && ls -d * && cd ../../..}
kicker_openocd_cfg_file := board/st_nucleo_f0.cfg

define create-kicker-board-rust-targets
.PHONY: .$1-$2-cargo-build
.$1-$2-cargo-build: motor-controller--dribbler motor-controller--wheel
	cd $1/ && \
	cargo build --target thumbv7em-none-eabihf --release --bin $2

$1/target/thumbv7em-none-eabihf/release/$2.bin: .$1-$2-cargo-build
	echo; \
	echo "Updating $1--$2 flat binary output:"; \
	elf_path="$1/target/thumbv7em-none-eabihf/release/$2"; \
	bin_path="$1/target/thumbv7em-none-eabihf/release/$2.bin"; \
	tmp_path="$1/target/thumbv7em-none-eabihf/release/$2.tmp.bin"; \
	arm-none-eabi-objcopy -Obinary $$$${elf_path} $$$${tmp_path}; \
	if ! cmp -s $$$${tmp_path} $$$${bin_path}; then \
	    echo "$2.bin has new changes!"; \
	    mv $$$${tmp_path} $$$${bin_path}; \
	else \
	    echo "$2.bin unchanged!"; \
	    rm $$$${tmp_path}; \
	fi; \
	echo; \

$1--$2: $1/target/thumbv7em-none-eabihf/release/$2.bin
$1--all:: $1--$2

$1--$2--run: $1--$2
	cd $1/ && \
	cargo run --release --bin $2

$1--$2--debug: $1--$2
	cd $1/ && \
	cargo build --release --bin $2 && \
	../util/program.sh $3 $1/target/thumbv7em-none-eabihf/release/$2
endef
$(foreach element,$(kicker_binaries),$(eval $(call create-kicker-board-rust-targets,kicker-board,$(element),$(kicker_openocd_cfg_file))))

kicker-board--clean:
	cd kicker-board && \
	cargo clean

############################
#  control board binaries  #
############################

.PHONY: control-board--all
control_binaries := ${shell cd control-board/src/bin && ls -d * && cd ../../..}
control_openocd_cfg_file := board/st_nucleo_h743zi.cfg

define create-control-board-rust-targets
.PHONY: .$1-$2-cargo-build
.$1-$2-cargo-build: kicker-board--kicker kicker-board--hwtest-coms motor-controller--wheel
	cd $1/ && \
	cargo build $(additional_control_cargo_flags) --target thumbv7em-none-eabihf --release --bin $2

.PHONY: .$1-$2-embed-hash
.$1-$2-embed-hash: .$1-$2-cargo-build
	python util/embed_img_hash.py --bin $1/target/thumbv7em-none-eabihf/release/$2

$1/target/thumbv7em-none-eabihf/release/$2.bin: .$1-$2-embed-hash
	echo; \
	echo "Updating $1--$2 flat binary output:"; \
	elf_path="$1/target/thumbv7em-none-eabihf/release/$2"; \
	bin_path="$1/target/thumbv7em-none-eabihf/release/$2.bin"; \
	tmp_path="$1/target/thumbv7em-none-eabihf/release/$2.tmp.bin"; \
	arm-none-eabi-objcopy -Obinary $$$${elf_path} $$$${tmp_path}; \
	if ! cmp -s $$$${tmp_path} $$$${bin_path}; then \
	    echo "$2.bin has new changes!"; \
	    mv $$$${tmp_path} $$$${bin_path}; \
	else \
	    echo "$2.bin unchanged!"; \
	    rm $$$${tmp_path}; \
	fi; \
	echo; \

$1--$2: $1/target/thumbv7em-none-eabihf/release/$2.bin
$1--all:: $1--$2

$1--$2--run: $1--$2
	cd $1/ && \
	cargo run $(additional_control_cargo_flags) --release --bin $2

$1--$2--debug: $1--$2
	cd $1/ && \
	../util/program.sh $3 target/thumbv7em-none-eabihf/release/$2
endef
$(foreach element,$(control_binaries),$(eval $(call create-control-board-rust-targets,control-board,$(element),$(control_openocd_cfg_file))))

control-board--clean: kicker-board--clean motor-controller--clean
	cd control-board && \
	cargo clean

##################
#  meta targets  #
##################

.PHONY: control
.DEFAULT_GOAL := control
control:: control-board--control--run

.PHONY: all
all:: kicker-board--all motor-controller--all control-board--all

.PHONY: clean
clean: software-communication--clean kicker-board--clean motor-controller--clean control-board--clean
