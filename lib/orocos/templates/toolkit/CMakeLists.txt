IF (<%= is_corba_enabled = (toolkit.corba_enabled? ? 1 : 0) ; is_corba_enabled %>)
    SET(CORBA_FILES <%= toolkit.name %>ToolkitC.h)
    ADD_CUSTOM_COMMAND(OUTPUT ${IDL_HEADERS}
	COMMAND ${TAO_IDL_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/TestToolkit.idl)
ENDIF (<%= is_corba_enabled %>)

ADD_LIBRARY(<%= toolkit.name %>-toolkit-${OROCOS_TARGET} SHARED
    <%= toolkit.name %>Toolkit.cpp ${CORBA_FILES})

