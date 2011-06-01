# Generated from orogen/lib/orogen/templates/typekit/mqueue/CMakeLists.txt

# OrocosRTT_PREFIX has been set by the pkg-config find modules:
find_package(Orocos-RTT REQUIRED rtt-transport-mqueue HINTS ${OrocosRTT_PREFIX}/lib/cmake/orocos-rtt)

pkg_check_modules(RTT_Typelib REQUIRED rtt_typelib)
include_directories(${RTT_Typelib_INCLUDE_DIRS})
link_directories(${RTT_Typelib_LIBRARY_DIRS})

# Set up generation of IDL files for MQueue support
IF (NOT OROCOS-RTT_MQUEUE_FOUND)
    MESSAGE(FATAL_ERROR "could not find development files for Orocos/MQueue. Please compile the RTT with MQueue support.")
ENDIF(NOT OROCOS-RTT_MQUEUE_FOUND)

set(MQueue_FILES "")

# Generate the MQueue transport
set(libname_mqueue <%= typekit.name %>-transport-mqueue-${OROCOS_TARGET})
add_library(${libname_mqueue} SHARED
    <%= impl.join("\n    ") %>
    ${MQueue_FILES})
target_link_libraries(${libname_mqueue}
    <%= typekit.name %>-typekit-${OROCOS_TARGET}
    <%= typekit.name %>-transport-typelib-${OROCOS_TARGET}
    ${OROCOS-RTT_MQUEUE_LIBRARIES}
    ${RTT_Typelib_LIBRARIES})

<%= typekit_deps = typekit.plugin('mqueue').dependencies(typekit);
    Generation.cmake_pkgconfig_link('mqueue', '${libname_mqueue}', typekit_deps) %>
target_link_libraries(${libname_mqueue} LINK_INTERFACE_LIBRARIES ${OROCOS-RTT_MQUEUE_LIBRARIES})

SET(PKG_CONFIG_FILE_MQueue ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>-transport-mqueue-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(<%= typekit.name %>-transport-mqueue.pc.in ${PKG_CONFIG_FILE_MQueue} @ONLY)

install(TARGETS ${libname_mqueue} LIBRARY DESTINATION lib/orocos/types)
install(FILES ${PKG_CONFIG_FILE_MQueue} DESTINATION lib/pkgconfig)
install(FILES
    <%= headers.join("\n   ") %>
    DESTINATION include/orocos/<%= typekit.name %>/transports/mqueue)

