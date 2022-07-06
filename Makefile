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

define make-test-target
.PHONY: test-$1
test-$1: .setup
	cd build/ && \
	make test-$1
.PHONY: test-$1-prog
test-$1-prog: test-$1
	cd build/ && \
	make test-$1-prog
all:: test-$1
endef

test_directories := ${shell cd test && ls -d *} 
$(foreach element,$(test_directories),$(eval $(call make-test-target,$(element))))

# .PHONY: aaa
# aaa:
# 	@if [ ! -z ${prog} ]; then \
# 		echo "program aaaa"; \
# 	fi

define make-stspin-target
.PHONY: stspin-$1
stspin-$1: override prog=
stspin-$1: override monitor=
stspin-$1: aaa
	@ cd stspin && \
	if [ ! -z ${monitor} ]; then \
		cargo run --release --bin $1 monitor; \
	elif [ ! -z ${prog} ]; then \
		cargo run --release --bin $1; \
	else \
		cargo build --release --bin $1; \
	fi
endef

stspin_directories := ${shell cd stspin/src/bin && ls -d *}
$(foreach element,$(stspin_directories),$(eval $(call make-stspin-target,$(element))))

.PHONY: stspin
stspin:
	echo $(foreach element,$(motor_directories), aaa/$(element))
#	@ cd stspin && \
#	cargo build --release && \
#	cp target/thumbv6m-none-eabi/release/ 

define make-motor-target
.PHONY: motor-$1
motor-$1: override prog=
motor-$1: override monitor=
motor-$1: aaa
	@ cd motor-board && \
	if [ ! -z ${monitor} ]; then \
		cargo run --bin $1 monitor; \
	elif [ ! -z ${prog} ]; then \
		cargo run --bin $1; \
	else \
		cargo build --bin $1; \
	fi
endef

motor_directories := ${shell cd motor-board/src/bin && ls -d *}
$(foreach element,$(motor_directories),$(eval $(call make-motor-target,$(element))))

# .PHONY: motor-board
# motor-board:
# 	export IS_MAKE=true && \
# 	cd motor-board && \
# 	IS_MAKE=true cargo build

# define make-stspin-target
# .PHONY: stspin-$1
# stspin-$1
