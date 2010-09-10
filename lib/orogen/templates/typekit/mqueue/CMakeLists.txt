# Generated from orogen/lib/orogen/templates/typekit/mqueue/CMakeLists.txt
IF (NOT OROCOS-RTT_FOUND)
  FIND_PACKAGE(OROCOS-RTT REQUIRED)
ENDIF (NOT OROCOS-RTT_FOUND)

# Set up generation of IDL files for MQueue support
IF (NOT OROCOS-RTT_MQUEUE_FOUND)
    MESSAGE(FATAL_ERROR "could not find development files for Orocos/MQueue")
ENDIF(NOT OROCOS-RTT_MQUEUE_FOUND)

set(MQueue_FILES "")

# Generate the MQueue transport
set(libname_mqueue <%= typekit.name %>-transport-mqueue-${OROCOS_TARGET})
rosbuild_add_library(${libname_mqueue}
    <%= impl.join("\n    ") %>
    ${MQueue_FILES})
target_link_libraries(${libname_mqueue}
    <%= typekit.name %>-typekit-${OROCOS_TARGET}
    ${OROCOS-RTT_MQUEUE_LIBRARIES})

<%= typekit_deps = typekit.plugin('mqueue').dependencies(typekit);
    Generation.cmake_pkgconfig_link('mqueue', '${libname_mqueue}', typekit_deps) %>
target_link_libraries(${libname_mqueue} LINK_INTERFACE_LIBRARIES ${OROCOS-RTT_MQUEUE_LIBRARIES})
