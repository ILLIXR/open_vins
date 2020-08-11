nproc=$(shell python3 -c 'import multiprocessing; print( max(multiprocessing.cpu_count() - 1, 1))')

.PHONY: plugin.dbg.so
plugin.dbg.so:
	mkdir -p build/ && \
	cd build/ && \
	cmake .. -DCMAKE_CXX_COMPILER=clang++ && \
	make "-j$(nproc)" && \
	cd .. && \
	rm -f  plugin.dbg.so && \
	ln -s build/ov_msckf/libslam2.so plugin.dbg.so && \
	true

.PHONY: plugin.opt.so
plugin.opt.so:
	$(error "This plugin does not yet support an optimized build")

.PHONY: clean
clean:
	touch build && rm -rf build *.so
