IF (NOT OrocosRTT_FOUND)
    FIND_PACKAGE(OrocosRTT REQUIRED)
ENDIF(NOT OrocosRTT_FOUND)

pkg_check_modules(OrocosCORBA orocos-rtt-corba-${OROCOS_TARGET})
IF(NOT OrocosCORBA_FOUND)
    IF (OrocosCORBA_FIND_REQUIRED)
	MESSAGE(FATAL_ERROR "RTT has not been built with CORBA support")
    ELSEIF(NOT OrocosCORBA_FIND_QUIETLY)
	MESSAGE(STATUS "RTT has not been built with CORBA support")
    ENDIF (OrocosCORBA_FIND_REQUIRED)

ELSE(NOT OrocosCORBA_FOUND)
    LIST(APPEND OROCOS_COMPONENT_INCLUDE ${OrocosCORBA_INCLUDE_DIRS})

    # First, find the actual implementation of the used ORB (TAO or OMNIORB)
    FIND_FILE( OrocosCORBA_config rtt/corba/rtt-corba-config.h ${OrocosRTT_INCLUDE_DIRS})
    FILE(STRINGS ${OrocosCORBA_config} CORBA_IMPLEMENTATION
        REGEX "define RTT_CORBA_IMPLEMENTATION ")
    STRING(STRIP ${CORBA_IMPLEMENTATION} CORBA_IMPLEMENTATION)
    STRING(REGEX MATCH "[^ ]+$" CORBA_IMPLEMENTATION ${CORBA_IMPLEMENTATION})

    # In the case of the TAO ORB, we add TAO_Strategies to be able to use
    # multiple communication methods
    IF (CORBA_IMPLEMENTATION STREQUAL "TAO")
        LIST(APPEND OrocosCORBA_LIBS TAO_Strategies)
    ENDIF (CORBA_IMPLEMENTATION STREQUAL "TAO")

    # ORB-specific configuration steps if the TOOLKIT component
    # is required
    STRING(REGEX MATCH "Toolkit" _orocos_corba_toolkit "${OrocosCORBA_FIND_COMPONENTS}")
    IF (_orocos_corba_toolkit)
        IF (CORBA_IMPLEMENTATION STREQUAL "TAO")
            MESSAGE(STATUS "Orocos uses the TAO ORB")

            FIND_PROGRAM(OrocosCORBA_IDL_EXECUTABLE tao_idl)
            OROCOS_PKGCONFIG(TAO_CosEvent ORBSVCS_FOUND ORBSVCS_INCLUDE_DIR ORBSVCS_DEFINES ORBSVCS_LINK_DIR ORBSVCS_LIBRARIES)

            IF (OrocosCORBA_IDL_EXECUTABLE AND ORBSVCS_FOUND)
                SET(OrocosCORBA_Toolkit_FOUND TRUE)
                ADD_DEFINITIONS(-D_REENTRANT)
                SET(OrocosCORBA_Toolkit_DEFINES ${ORBSVCS_DEFINES})
                # The ${OrocosRTT_INCLUDE_DIRS}/rtt part is a workaround for RTT's
                # includes brokenness
                SET(OrocosCORBA_Toolkit_INCLUDE_DIR "${ORBSVCS_INCLUDE_DIR};${OrocosCORBA_INCLUDE_DIRS}")
                SET(OrocosCORBA_IDL ${OrocosCORBA_IDL_EXECUTABLE})
            ELSE(OrocosCORBA_IDL_EXECUTABLE AND ORBSVCS_FOUND)
                IF(NOT OrocosCORBA_FIND_QUIETLY)
                    MESSAGE(STATUS "cannot find tao_idl or associated development files")
                ENDIF(NOT OrocosCORBA_FIND_QUIETLY)
            ENDIF (OrocosCORBA_IDL_EXECUTABLE AND ORBSVCS_FOUND)

        ELSEIF (CORBA_IMPLEMENTATION STREQUAL "OMNIORB")
            MESSAGE(STATUS "Orocos uses the OmniORB")

            FIND_PROGRAM(OrocosCORBA_IDL_EXECUTABLE omniidl)
            IF (OrocosCORBA_IDL_EXECUTABLE)
                SET(OrocosCORBA_Toolkit_FOUND TRUE)
                # The ${OrocosRTT_INCLUDE_DIRS}/rtt part is a workaround for RTT's
                # includes brokenness
                SET(OrocosCORBA_Toolkit_INCLUDE_DIR "${OROCOS_RTT_INCLUDE_DIRS}/rtt;${OROCOS_RTT_INCLUDE_DIRS}/rtt/corba")
                SET(OrocosCORBA_IDL ${OrocosCORBA_IDL_EXECUTABLE} -bcxx -Wba -Wbh=C.h -Wbs=C.cpp -Wbd=DynSK.cpp)
            ELSE(OrocosCORBA_IDL_EXECUTABLE)
                IF(NOT OrocosCORBA_FIND_QUIETLY)
                    MESSAGE(STATUS "cannot find omniidl or associated development files")
                ENDIF(NOT OrocosCORBA_FIND_QUIETLY)
            ENDIF (OrocosCORBA_IDL_EXECUTABLE)
        ELSE (CORBA_IMPLEMENTATION STREQUAL "TAO")
            MESSAGE(FATAL_ERROR "unknown Corba implementation ${CORBA_IMPLEMENTATION}")
        ENDIF (CORBA_IMPLEMENTATION STREQUAL "TAO")
    ENDIF (_orocos_corba_toolkit)
ENDIF(NOT OrocosCORBA_FOUND)

