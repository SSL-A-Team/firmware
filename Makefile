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


steval-spin3201-6step: setup
	cd build/ && \
	make steval-spin3201-6step

steval-spin3201-6step-prog: steval-spin3201-6step
	cd build/ && \
	make steval-spin3201-6step-prog

steval-spin3201-6step-gdb: steval-spin3201-6step
	cd build/ && \
	make steval-spin3201-6step-gdb

