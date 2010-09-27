EXTRA_CMAKE_FLAGS=-DCMAKE_PREFIX_PATH=`rospack find rtt`/install

include $(shell rospack find mk)/cmake.mk

wipe: clean
	rm -rf install lib bin
	rm -rf installed
