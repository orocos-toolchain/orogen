# Generated from orogen/lib/orogen/templates/typekit/corba/build.cmake

<% typekit_deps = typekit.plugin('corba').dependencies %>

find_package(OrocosCORBA REQUIRED COMPONENTS Typekit)
add_definitions(${OrocosCORBA_DEFINES})
include_directories(${OrocosCORBA_INCLUDE_DIRS})

<%= Generation.cmake_pkgconfig_require(typekit_deps, 'corba') %>

IF (NOT OrocosCORBA_Typekit_FOUND)
    MESSAGE(FATAL_ERROR "could not find development files for Orocos/CORBA")
ENDIF(NOT OrocosCORBA_Typekit_FOUND)

set(CORBA_FILES ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>TypesC.cpp)
if(CORBA_IMPLEMENTATION STREQUAL "OMNIORB")
    list(APPEND CORBA_FILES ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>TypesDynSK.cpp)
endif(CORBA_IMPLEMENTATION STREQUAL "OMNIORB")

add_custom_command(OUTPUT ${CORBA_FILES}
    COMMAND ${OrocosCORBA_IDL} -Wbkeep_inc_path
        <%= typekit.used_typekits.find_all { |tk| !tk.virtual? }.map { |tk| "-I" + File.join(tk.pkg.prefix, "include", "orocos") }.join(" ") %>
        ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>Types.idl
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    DEPENDS ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>Types.idl)

add_definitions(${OrocosCORBA_Typekit_DEFINES})
include_directories(${OrocosCORBA_Typekit_INCLUDE_DIRS})

# Generate the CORBA transport
set(libname_corba <%= typekit.name %>-transport-corba)
orocos_typekit(${libname_corba}
  <%= impl.sort.map { |filepath| '${CMAKE_CURRENT_LIST_DIR}/'+ (File.basename filepath) }.join("\n    ") %>
    ${CORBA_FILES})
target_link_libraries(${libname_corba}
    <%= typekit.name %>-typekit ${OROCOS-RTT_CORBA_LIBRARIES})
set_target_properties(${libname_corba} PROPERTIES COMPILE_FLAGS "-Wno-unused-variable")
<%= Generation.cmake_pkgconfig_link('corba', '${libname_corba}', typekit_deps) %>
target_link_libraries(${libname_corba} LINK_INTERFACE_LIBRARIES ${OROCOS-RTT_CORBA_LIBRARIES})
set_target_properties(${libname_corba} PROPERTIES INTERFACE_LINK_LIBRARIES "${OROCOS-RTT_CORBA_LIBRARIES}")

SET(PKG_CONFIG_FILE_CORBA ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>-transport-corba-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>-transport-corba.pc.in ${PKG_CONFIG_FILE_CORBA} @ONLY)

install(FILES ${PKG_CONFIG_FILE_CORBA} DESTINATION lib/pkgconfig)
install(FILES
    <%= headers.map { |filepath| '${CMAKE_CURRENT_LIST_DIR}/'+filepath }.join("\n   ") %>
    ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>Types.idl
    ${CMAKE_CURRENT_LIST_DIR}/<%= typekit.name %>TypesC.h
    DESTINATION include/orocos/<%= typekit.name %>/transports/corba)

