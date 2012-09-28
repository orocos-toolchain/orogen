ifdef ROS_ROOT
export MALLOC_CHECK_=0
default: install
include $(shell rospack find rtt)/../env.mk
install: 
	rake setup[-DOROCOS_TARGET=$(OROCOS_TARGET)]
else
$(warning This Makefile only works with ROS rosmake. Without rosmake, create a build directory and run cmake ..)
endif
