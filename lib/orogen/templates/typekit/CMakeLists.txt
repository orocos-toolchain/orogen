# Generated from orogen/lib/orogen/templates/typekit/CMakeLists.txt

##
# Header for typegen-generated typekits
#
<% if typekit.standalone? %>
if(NOT ${PROJECT_NAME} STREQUAL "<%= typekit.name %>")
  project("<%= typekit.name %>-typekit")
  cmake_minimum_required(VERSION 2.6)
endif()

# In Orogen components, the build target is specified at generation time
# But we allow to override in case of typegen/USE_OROCOS_RTT:
if (NOT OROCOS_TARGET)
  set(OROCOS_TARGET "<%= typekit.orocos_target %>")
endif()
# If UseOrocos-RTT.cmake is used, install libs in /target/package subdir in order to allow
# multi-target/package installs.
if ( USE_OROCOS_RTT )
  if ( NOT OROCOS_SUFFIX )
    set (OROCOS_SUFFIX "/${OROCOS_TARGET}")
  endif()
  set(OROCOS_PKG_DIR "${OROCOS_SUFFIX}/<%= typekit.name %>")
endif()

# Enable -Wall for compilers that know it
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-Wall" CXX_SUPPORTS_WALL)
if (CXX_SUPPORTS_WALL)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif (CXX_SUPPORTS_WALL)

# Define the necessary RPath information for all binaries. That can be stripped
# later
option(WITH_RPATH "enables or disables embedding RPath information in binaries" ON)
if(WITH_RPATH)
    include(RPATHHandling)
    CMAKE_USE_FULL_RPATH("${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/orocos:${CMAKE_INSTALL_PREFIX}/lib/orocos${OROCOS_PKG_DIR}/types")
endif(WITH_RPATH)

# Set the build type to debug by default
IF ( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Debug )
ENDIF ( NOT CMAKE_BUILD_TYPE )

find_package(Orocos-RTT 2 REQUIRED)
include(${OROCOS-RTT_USE_FILE})

<% end %>
# END if typekit.standalone?
##

include(${CMAKE_CURRENT_SOURCE_DIR}/build.cmake)
