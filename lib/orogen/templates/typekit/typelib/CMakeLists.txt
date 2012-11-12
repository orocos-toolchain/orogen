# Generated from orogen/lib/orogen/templates/typekit/typelib/CMakeLists.txt

# This policy indicates that the code below is cmake 2.6 compliant. This is necessary
# To properly escape the add_definitions arguments.
cmake_policy(VERSION 2.6)

<% typekit_deps = typekit.plugin('typelib').dependencies %>
<%= Generation.cmake_pkgconfig_require(typekit_deps, 'typelib') %>

orogen_pkg_check_modules(TYPELIB REQUIRED typelib)
include_directories(${TYPELIB_INCLUDE_DIRS})
link_directories(${TYPELIB_LIBRARY_DIRS})

orogen_pkg_check_modules(RTT_Typelib REQUIRED rtt-typelib)
include_directories(${RTT_Typelib_INCLUDE_DIRS})
link_directories(${RTT_Typelib_LIBRARY_DIRS})

set(TYPEKIT_REGISTRY ${CMAKE_INSTALL_PREFIX}/share/orogen/<%= typekit.name %>.tlb)
add_definitions(-DTYPEKIT_REGISTRY="${TYPEKIT_REGISTRY}")
set(libname_typelib <%= typekit.name %>-transport-typelib-${OROCOS_TARGET})
add_library(${libname_typelib} SHARED
    <%= impl.join("\n    ") %>)

target_link_libraries(${libname_typelib}
    ${TYPELIB_LIBRARIES}
    ${OrocosRTT_LIBRARIES}
    ${RTT_Typelib_LIBRARIES}
    <% if !typekit.opaques.empty? %>
    <%= typekit.name %>-typekit-${OROCOS_TARGET}
    <% end %>)

<%= Generation.cmake_pkgconfig_link('typelib', '${libname_typelib}', typekit_deps) %>

# rospack support
if ( ROS_ROOT )
        rosbuild_find_ros_package( roslib )
        include_directories( ${roslib_PACKAGE_PATH}/include )
        find_library(ROS_LIB roslib ${roslib_PACKAGE_PATH}/lib )
        TARGET_LINK_LIBRARIES( ${libname_typelib} ${ROS_LIB} )
        ADD_DEFINITIONS( -DHAS_ROSLIB )
endif( ROS_ROOT )

SET(PKG_CONFIG_FILE_TYPELIB ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>-transport-typelib-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(<%= typekit.name %>-transport-typelib.pc.in ${PKG_CONFIG_FILE_TYPELIB} @ONLY)

install(FILES ${PKG_CONFIG_FILE_TYPELIB} DESTINATION lib/pkgconfig)
install(FILES
    TransportPlugin.hpp
    DESTINATION include/orocos/<%= typekit.name %>/transports/typelib)
install(TARGETS ${libname_typelib} LIBRARY DESTINATION lib/orocos/types)

