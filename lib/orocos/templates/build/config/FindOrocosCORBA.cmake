IF (NOT OrocosRTT_FOUND)
    FIND_PACKAGE(OrocosRTT REQUIRED)
ENDIF(NOT OrocosRTT_FOUND)


FIND_FILE( OrocosCORBA_FOUND rtt/corba/ControlTaskProxy.hpp ${OROCOS_RTT_INCLUDE_DIRS})
IF(NOT OrocosCORBA_FOUND)
    IF (OrocosCORBA_FIND_REQUIRED)
	MESSAGE(FATAL_ERROR "RTT has not been built with CORBA support")
    ELSEIF(NOT OrocosCORBA_FIND_QUIETLY)
	MESSAGE(STATUS "RTT has not been built with CORBA support")
    ENDIF (OrocosCORBA_FIND_REQUIRED)

ELSE(NOT OrocosCORBA_FOUND)

    # TAO-specific configuration steps if the TOOLKIT component
    # is required
    STRING(REGEX MATCH "Toolkit" _orocos_corba_toolkit "${OrocosCORBA_FIND_COMPONENTS}")
    IF (_orocos_corba_toolkit)
	FIND_PROGRAM(OrocosCORBA_TAO_IDL_EXECUTABLE tao_idl)
	PKGCONFIG(TAO_CosEvent ORBSVCS_FOUND ORBSVCS_INCLUDE_DIR ORBSVCS_DEFINES ORBSVCS_LINK_DIR ORBSVCS_LIBRARIES)

	IF (OrocosCORBA_TAO_IDL_EXECUTABLE AND ORBSVCS_FOUND)
	    SET(OrocosCORBA_Toolkit_FOUND TRUE)
	    ADD_DEFINITIONS(-D_REENTRANT)
	    SET(OrocosCORBA_Toolkit_DEFINES ${ORBSVCS_DEFINES})
	    # The ${OrocosRTT_INCLUDE_DIRS}/rtt part is a workaround for RTT's
	    # includes brokenness
	    SET(OrocosCORBA_Toolkit_INCLUDE_DIR ${ORBSVCS_INCLUDE_DIR} ${OROCOS_RTT_INCLUDE_DIRS}/rtt)
	ELSE(OrocosCORBA_TAO_IDL_EXECUTABLE AND ORBSVCS_FOUND)
	    IF(NOT OrocosCORBA_FIND_QUIETLY)
		MESSAGE(STATUS "cannot find tao_idl or associated development files")
	    ENDIF(NOT OrocosCORBA_FIND_QUIETLY)
	ENDIF (OrocosCORBA_TAO_IDL_EXECUTABLE AND ORBSVCS_FOUND)
    ENDIF (_orocos_corba_toolkit)

ENDIF(NOT OrocosCORBA_FOUND)

