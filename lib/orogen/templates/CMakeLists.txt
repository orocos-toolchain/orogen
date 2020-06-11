cmake_minimum_required(VERSION 3.1)
project(<%= project.name %> VERSION <%= project.version %>)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/config")
include(<%= project.name %>Base)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

