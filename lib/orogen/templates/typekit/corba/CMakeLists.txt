# Generated from orogen/lib/orogen/templates/typekit/corba/CMakeLists.txt

find_package(OrocosCORBA REQUIRED COMPONENTS Typekit)
INCLUDE_DIRECTORIES(${OrocosCORBA_INCLUDE_DIRS})
ADD_DEFINITIONS(${OrocosCORBA_DEFINES})

include_directories(${OrocosCORBA_INCLUDE_DIRS})
add_definitions(${OrocosCORBA_CFLAGS_OTHER})
link_directories(${OrocosCORBA_LIBRARY_DIRS})

# Set up generation of IDL files for CORBA support
IF (NOT OrocosCORBA_Typekit_FOUND)
    MESSAGE(FATAL_ERROR "could not find development files for Orocos/CORBA")
ENDIF(NOT OrocosCORBA_Typekit_FOUND)

set(CORBA_FILES ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>TypesC.cpp)
if(CORBA_IMPLEMENTATION STREQUAL "OMNIORB")
    list(APPEND CORBA_FILES ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>TypesDynSK.cpp)
endif(CORBA_IMPLEMENTATION STREQUAL "OMNIORB")

add_custom_command(OUTPUT ${CORBA_FILES}
    COMMAND ${OrocosCORBA_IDL} -Wbkeep_inc_path
        <%= typekit.used_typekits.map { |tk| "-I" + File.join(tk.pkg.prefix, "include", "orocos") }.join(" ") %>
        ${CMAKE_CURRENT_SOURCE_DIR}/<%= typekit.name %>Types.idl
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/<%= typekit.name %>Types.idl)

add_definitions(${OrocosCORBA_Typekit_DEFINES})
include_directories(${OrocosCORBA_Typekit_INCLUDE_DIRS})

# Generate the CORBA transport
set(libname_corba <%= typekit.name %>-transport-corba-${OROCOS_TARGET})
add_library(${libname_corba} SHARED
    <%= relatives = []
        impl.each do |file| 
	    relatives << File.basename(file)
        end
        relatives.sort.join("\n    ") %>
    ${CORBA_FILES})
target_link_libraries(${libname_corba}
    <%= typekit.name %>-typekit-${OROCOS_TARGET}
    ${OrocosCORBA_LIBRARIES})

<%= typekit_deps = typekit.plugin('corba').dependencies(typekit);
    Generation.cmake_pkgconfig_link_corba('${libname_corba}', typekit_deps) %>
target_link_libraries(${libname_corba} LINK_INTERFACE_LIBRARIES ${OrocosCORBA_LIBRARIES})

SET(PKG_CONFIG_FILE_CORBA ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>-transport-corba-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(<%= typekit.name %>-transport-corba.pc.in ${PKG_CONFIG_FILE_CORBA} @ONLY)

install(TARGETS ${libname_corba} LIBRARY DESTINATION lib/orocos/types)
install(FILES ${PKG_CONFIG_FILE_CORBA} DESTINATION lib/pkgconfig)
install(FILES
    <%= relatives = []
        headers.each do |file| 
	    relatives << File.basename(file)
        end
        relatives.sort.join("\n    ") %>
    Convertions.hpp
    <%= typekit.name %>Types.idl
    ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>TypesC.h
    DESTINATION include/orocos/<%= typekit.name %>/transports/corba)

