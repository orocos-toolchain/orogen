# Generated from orogen/lib/orogen/templates/typekit/typelib/build.cmake

# This policy indicates that the code below is cmake 2.6 compliant. This is necessary
# To properly escape the add_definitions arguments.
cmake_policy(VERSION 2.6)

<% typekit_deps = typekit.plugin('typelib').dependencies %>
<%= Generation.cmake_pkgconfig_require(typekit_deps, 'typelib') %>
orogen_pkg_check_modules(typelib REQUIRED typelib)
include_directories(${typelib_INCLUDE_DIRS})
link_directories(${typelib_LIBRARY_DIRS})

orocos_find_package(rtt_typelib REQUIRED)
include_directories(${rtt_typelib_INCLUDE_DIRS})
link_directories(${rtt_typelib_LIBRARY_DIRS})

set(TYPEKIT_REGISTRY ${CMAKE_INSTALL_PREFIX}/share/orogen/<%= typekit.name %>.tlb)
add_definitions(-DTYPEKIT_REGISTRY="${TYPEKIT_REGISTRY}")
set(libname_typelib <%= typekit.name %>-transport-typelib)
orocos_typekit(${libname_typelib}
    <%= impl.join("\n    ") %>)

target_link_libraries(${libname_typelib}
    ${typelib_LIBRARIES}
    ${rtt_typelib_LIBRARIES}
    <% if !typekit.opaques.empty? %>
    <%= typekit.name %>-typekit
    <% end %>)

<%= Generation.cmake_pkgconfig_link('typelib', '${libname_typelib}', typekit_deps) %>

SET(PKG_CONFIG_FILE_TYPELIB ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>-transport-typelib-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>-transport-typelib.pc.in ${PKG_CONFIG_FILE_TYPELIB} @ONLY)

install(FILES ${PKG_CONFIG_FILE_TYPELIB} DESTINATION lib/pkgconfig)
install(FILES
    ${CMAKE_CURRENT_LIST_DIR}/TransportPlugin.hpp
    DESTINATION include/orocos/<%= typekit.name %>/transports/typelib)
