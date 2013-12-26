# Generated from orogen/lib/orogen/templates/typekit/corba/CMakeLists.txt

<% typekit_deps = typekit.plugin('corba').dependencies(typekit); %>

find_package(OrocosCORBA REQUIRED COMPONENTS Typekit)
add_definitions(${OrocosCORBA_DEFINES})
include_directories(${OrocosCORBA_INCLUDE_DIRS})

<%= Generation.cmake_pkgconfig_require(typekit_deps, 'corba') %>

IF (NOT OrocosCORBA_Typekit_FOUND)
    MESSAGE(FATAL_ERROR "could not find development files for Orocos/CORBA")
ENDIF(NOT OrocosCORBA_Typekit_FOUND)

set(CORBA_FILES ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>TypesC.cpp)
if(CORBA_IMPLEMENTATION STREQUAL "OMNIORB")
    list(APPEND CORBA_FILES ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>TypesDynSK.cpp)
endif(CORBA_IMPLEMENTATION STREQUAL "OMNIORB")

add_custom_command(OUTPUT ${CORBA_FILES}
    COMMAND ${OrocosCORBA_IDL} -Wbkeep_inc_path
        <%= typekit.used_typekits.find_all { |tk| !tk.virtual? }.map { |tk| "-I" + File.join(tk.pkg.prefix, "include", "orocos") }.join(" ") %>
        ${CMAKE_CURRENT_SOURCE_DIR}/<%= typekit.name %>Types.idl
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/<%= typekit.name %>Types.idl)

add_definitions(${OrocosCORBA_Typekit_DEFINES})
include_directories(${OrocosCORBA_Typekit_INCLUDE_DIRS})

# Generate the CORBA transport
set(libname_corba <%= typekit.name %>-transport-corba-${OROCOS_TARGET})
orocos_typekit(${libname_corba}
    <%= impl.join("\n    ") %>
    ${CORBA_FILES})
target_link_libraries(${libname_corba}
    <%= typekit.name %>-typekit-${OROCOS_TARGET} ${OROCOS-RTT_CORBA_LIBRARIES})

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

