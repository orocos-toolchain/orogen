IF (<%= is_corba_enabled = (toolkit.corba_enabled? ? 1 : 0) ; is_corba_enabled %>)
    IF (NOT OrocosCORBA_Toolkit_FOUND)
	MESSAGE(FATAL_ERROR "could not find development files for Orocos/CORBA")
    ENDIF(NOT OrocosCORBA_Toolkit_FOUND)

    SET(CORBA_FILES ${CMAKE_CURRENT_BINARY_DIR}/<%= toolkit.name %>ToolkitC.cpp)
    ADD_CUSTOM_COMMAND(OUTPUT ${CORBA_FILES}
	COMMAND ${OrocosCORBA_TAO_IDL_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/TestToolkit.idl
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/TestToolkit.idl)

    ADD_DEFINITIONS(${OrocosCORBA_Toolkit_DEFINES})
    INCLUDE_DIRECTORIES(BEFORE ${CMAKE_CURRENT_BINARY_DIR})
    INCLUDE_DIRECTORIES(${OrocosCORBA_Toolkit_INCLUDE_DIR})
ENDIF (<%= is_corba_enabled %>)

SET(libname <%= toolkit.name %>-toolkit-${OROCOS_TARGET})
ADD_LIBRARY(${libname} SHARED
    <%= toolkit.name %>Toolkit.cpp ${CORBA_FILES})

IF (<%= is_corba_enabled %>)
    LINK_DIRECTORIES(${OROCOS_RTT_LINK_DIRS})
    TARGET_LINK_LIBRARIES(${libname} orocos-rtt-corba-${OROCOS_TARGET})
ENDIF (<%= is_corba_enabled %>)

