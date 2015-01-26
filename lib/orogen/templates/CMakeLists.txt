PROJECT(<%= project.name %>)
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/config")
INCLUDE(<%= project.name %>Base)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

