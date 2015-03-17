# Generated from orogen/lib/orogen/templates/typekit/mqueue/build.cmake

if(NOT APPLE)

# OrocosRTT_PREFIX has been set by the pkg-config find modules:
find_package(Orocos-RTT REQUIRED rtt-transport-mqueue HINTS ${OrocosRTT_PREFIX}/lib/cmake/orocos-rtt)

<% typekit_deps = typekit.plugin('mqueue').dependencies %>
<%= Generation.cmake_pkgconfig_require(typekit_deps, 'mqueue') %>

orocos_find_package(rtt_typelib REQUIRED)
include_directories(${rtt_typelib_INCLUDE_DIRS})
link_directories(${rtt_typelib_LIBRARY_DIRS})

# Set up generation of IDL files for MQueue support
IF (NOT OROCOS-RTT_MQUEUE_FOUND)
    MESSAGE(FATAL_ERROR "could not find development files for Orocos/MQueue. Please compile the RTT with MQueue support.")
ENDIF(NOT OROCOS-RTT_MQUEUE_FOUND)

set(MQueue_FILES "")

# Generate the MQueue transport
set(libname_mqueue <%= typekit.name %>-transport-mqueue)
orocos_typekit(${libname_mqueue} SHARED
    <%= impl.map { |filepath| '${CMAKE_CURRENT_LIST_DIR}/'+(File.basename filepath) }.join("\n    ") %>
    ${MQueue_FILES})
target_link_libraries(${libname_mqueue}
    <%= typekit.name %>-typekit
    <%= typekit.name %>-transport-typelib
    ${OROCOS-RTT_MQUEUE_LIBRARIES}
    ${rtt_typelib_LIBRARIES})

<%= Generation.cmake_pkgconfig_link('mqueue', '${libname_mqueue}', typekit_deps) %>

SET(PKG_CONFIG_FILE_MQueue ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>-transport-mqueue-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>-transport-mqueue.pc.in ${PKG_CONFIG_FILE_MQueue} @ONLY)

install(FILES ${PKG_CONFIG_FILE_MQueue} DESTINATION lib/pkgconfig)
install(FILES
    <%= headers.map { |filepath| '${CMAKE_CURRENT_LIST_DIR}/'+(File.basename filepath) }.join("\n   ") %>
    DESTINATION include/orocos/<%= typekit.name %>/transports/mqueue)
else()
message(WARNING "Mqueue transport is not supported on OSX platforms")
endif()
