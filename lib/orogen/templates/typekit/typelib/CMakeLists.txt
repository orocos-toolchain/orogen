# Generated from orogen/lib/orogen/templates/typekit/typelib/CMakeLists.txt

# This policy indicates that the code below is cmake 2.6 compliant. This is necessary
# To properly escape the add_definitions arguments.
cmake_policy(VERSION 2.6)

<% typekit_deps = typekit.plugin('typelib').dependencies(typekit); %>
<%= Generation.cmake_pkgconfig_require(typekit_deps, 'typelib') %>
orogen_pkg_check_modules(TYPELIB REQUIRED typelib)
include_directories(${TYPELIB_INCLUDE_DIRS})
link_directories(${TYPELIB_LIBRARY_DIRS})

orocos_find_package(rtt_typelib REQUIRED)
include_directories(${rtt_typelib_INCLUDE_DIRS})
link_directories(${rtt_typelib_LIBRARY_DIRS})

set(TYPEKIT_REGISTRY ${CMAKE_INSTALL_PREFIX}/share/orogen/<%= typekit.name %>.tlb)
add_definitions(-DTYPEKIT_REGISTRY="${TYPEKIT_REGISTRY}")
set(libname_typelib <%= typekit.name %>-transport-typelib-${OROCOS_TARGET})
orocos_typekit(${libname_typelib}
    <%= impl.join("\n    ") %>)

target_link_libraries(${libname_typelib}
    ${TYPELIB_LIBRARIES}
    ${rtt_typelib_LIBRARIES}
    <% if !typekit.opaques.empty? %>
    <%= typekit.name %>-typekit-${OROCOS_TARGET}
    <% end %>)

<%= Generation.cmake_pkgconfig_link('typelib', '${libname_typelib}', typekit_deps) %>

SET(PKG_CONFIG_FILE_TYPELIB ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>-transport-typelib-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(<%= typekit.name %>-transport-typelib.pc.in ${PKG_CONFIG_FILE_TYPELIB} @ONLY)

install(FILES ${PKG_CONFIG_FILE_TYPELIB} DESTINATION lib/pkgconfig)
install(FILES
    TransportPlugin.hpp
    DESTINATION include/orocos/<%= typekit.name %>/transports/typelib)
install(TARGETS ${libname_typelib} LIBRARY DESTINATION lib/orocos${OROCOS_PKG_DIR}/types)

