.PHONY: all

.PHONY: setup
setup:
	mkdir -p build/ && \
	cd build && \
	cmake .. \

.PHONY: clean
clean:
	rm -rf build/

test_directories := ${shell cd test && ls -d * && cd ..} 

define make-test-target
.PHONY: test-$1
test-$1: setup
	cd build/ && \
	make test-$1
all:: test-$1
endef
$(foreach element,$(test_directories),$(eval $(call make-test-target,$(element))))

define make-test-prog-target
.PHONY: test-$1-prog
test-$1-prog: test-$1
	cd build/ && \
	make test-$1-prog
endef
$(foreach element,$(test_directories),$(eval $(call make-test-prog-target,$(element))))

steval-spin3201-6step: setup
	cd build/ && \
	make steval-spin3201-6step

steval-spin3201-6step-prog: steval-spin3201-6step
	cd build/ && \
	make steval-spin3201-6step-prog

steval-spin3201-6step-gdb: steval-spin3201-6step
	cd build/ && \
	make steval-spin3201-6step-gdb
