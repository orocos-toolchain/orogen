ifdef ROS_ROOT
default: install
include $(shell rosstack find orocos_toolchain_ros)/env.mk
install: 
	rake
else
$(warning This Makefile only works with ROS rosmake. Without rosmake, create a build directory and run cmake ..)
endif
