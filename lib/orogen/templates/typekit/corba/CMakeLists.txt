# Generated from orogen/lib/orogen/templates/typekit/corba/CMakeLists.txt

<% typekit_deps = typekit.plugin('corba').dependencies(typekit); %>

find_package(Orocos-RTT 2 REQUIRED)

if(NOT OROCOS-RTT_CORBA_FOUND)
       message(FATAL "Could not find Orocos RTT Corba bindings")
endif()
<%= Generation.cmake_pkgconfig_require(typekit_deps, 'corba') %>

set(CORBA_FILES ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>TypesC.cpp)
if(CORBA_IMPLEMENTATION STREQUAL "OMNIORB")
    list(APPEND CORBA_FILES ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>TypesDynSK.cpp)
endif(CORBA_IMPLEMENTATION STREQUAL "OMNIORB")

add_custom_command(OUTPUT ${CORBA_FILES}
    COMMAND ${OrocosCORBA_IDL} -Wbkeep_inc_path
        <%= typekit.used_typekits.find_all { |tk| !tk.virtual? }.map { |tk| "-I" + File.join(tk.pkg.prefix, "include", "orocos") }.join(" ") %>
        ${CMAKE_CURRENT_SOURCE_DIR}/<%= typekit.name %>Types.idl
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/<%= typekit.name %>Types.idl)

# Generate the CORBA transport
set(libname_corba <%= typekit.name %>-transport-corba-${OROCOS_TARGET})
orocos_typekit(${libname_corba}
    <%= impl.join("\n    ") %>
    ${CORBA_FILES})
target_link_libraries(${libname_corba}
    <%= typekit.name %>-typekit-${OROCOS_TARGET} ${OROCOS-RTT_CORBA_LIBRARIES})

if(WITH_RPATH AND APPLE)
  set_target_properties( ${libname_corba} PROPERTIES
    INSTALL_NAME_DIR "@rpath"
    LINK_FLAGS "-Wl,-rpath ${CMAKE_INSTALL_PREFIX}/lib -rpath ${CMAKE_INSTALL_PREFIX}/lib/orocos -rpath ${CMAKE_INSTALL_PREFIX}/lib/orocos${OROCOS_PKG_DIR}/types")
endif()

<%= Generation.cmake_pkgconfig_link('corba', '${libname_corba}', typekit_deps) %>
target_link_libraries(${libname_corba} LINK_INTERFACE_LIBRARIES ${OROCOS-RTT_CORBA_LIBRARIES})

SET(PKG_CONFIG_FILE_CORBA ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>-transport-corba-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(<%= typekit.name %>-transport-corba.pc.in ${PKG_CONFIG_FILE_CORBA} @ONLY)

install(FILES ${PKG_CONFIG_FILE_CORBA} DESTINATION lib/pkgconfig)
install(FILES
    <%= headers.join("\n   ") %>
    Convertions.hpp
    <%= typekit.name %>Types.idl
    ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>TypesC.h
    DESTINATION include/orocos/<%= typekit.name %>/transports/corba)

