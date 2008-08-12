# Corba-related generation: IDL, Corba-to-Orocos conversion code. This is done
# only if the user does not have disabled Corba support by calling
# #disable_corba
IF (<%= is_corba_enabled = (toolkit.corba_enabled? ? 1 : 0) ; is_corba_enabled %>)
    IF (NOT OrocosCORBA_Toolkit_FOUND)
	MESSAGE(FATAL_ERROR "could not find development files for Orocos/CORBA")
    ENDIF(NOT OrocosCORBA_Toolkit_FOUND)

    SET(CORBA_FILES ${CMAKE_CURRENT_BINARY_DIR}/<%= toolkit.name %>ToolkitC.cpp)
    ADD_CUSTOM_COMMAND(OUTPUT ${CORBA_FILES}
	COMMAND ${OrocosCORBA_TAO_IDL_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/<%= toolkit.name %>Toolkit.idl
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/<%= toolkit.name %>Toolkit.idl)

    ADD_DEFINITIONS(${OrocosCORBA_Toolkit_DEFINES})
    INCLUDE_DIRECTORIES(BEFORE ${CMAKE_CURRENT_BINARY_DIR})
    INCLUDE_DIRECTORIES(${OrocosCORBA_Toolkit_INCLUDE_DIR})
    INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/<%= toolkit.name %>Toolkit.idl
	DESTINATION include/orocos/toolkit
	RENAME <%= toolkit.name %>.idl)
ENDIF (<%= is_corba_enabled %>)

# Add the include directories to the component dependencies to the include path
<% component.used_libraries.each do |pkg| %>
pkg_check_modules(<%= pkg.name %> REQUIRED <%= pkg.name %>)
INCLUDE_DIRECTORIES(${<%= pkg.name %>_INCLUDE_DIRS})
<% end %>

# Generate the toolkit shared library
SET(libname <%= toolkit.name %>-toolkit-${OROCOS_TARGET})
ADD_LIBRARY(${libname} SHARED <%= toolkit.name %>Toolkit.cpp ${CORBA_FILES})
INSTALL(TARGETS ${libname} LIBRARY DESTINATION lib/orocos)
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/<%= toolkit.name %>ToolkitTypes.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/<%= toolkit.name %>Toolkit.hpp
    DESTINATION include/orocos/toolkit)

IF (<%= is_corba_enabled %>)
    LINK_DIRECTORIES(${OROCOS_RTT_LINK_DIRS})
    TARGET_LINK_LIBRARIES(${libname} orocos-rtt-corba-${OROCOS_TARGET})
ENDIF (<%= is_corba_enabled %>)

# Generate the pkg-config file and install it
SET(PKG_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/<%= toolkit.name %>-toolkit-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(<%= toolkit.name %>-toolkit.pc.in ${PKG_CONFIG_FILE} @ONLY)
INSTALL(FILES ${PKG_CONFIG_FILE} DESTINATION lib/pkgconfig)

SET(TLB_FILE ${CMAKE_CURRENT_SOURCE_DIR}/<%= toolkit.name %>.tlb)
INSTALL(FILES ${TLB_FILE} DESTINATION share/orogen)

