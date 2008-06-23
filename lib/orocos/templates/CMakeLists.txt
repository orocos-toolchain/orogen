PROJECT(<%= component.name %>)
cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/config")
INCLUDE(OrocosComponent)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

