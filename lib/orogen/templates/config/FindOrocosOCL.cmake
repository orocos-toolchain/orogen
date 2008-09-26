# Find parameters to use the Orocos Component Library
#
# If OCL_INSTALL is defined, it is the prefix where OCL is installed
# 
# This module defines
# OrocosOCL_FOUND is set to true if OCL is found, in which case the following
# variables are defined:
#    OrocosOCL_INCLUDE_DIR
#    OrocosOCL_LINK_DIR
#    OrocosOCL_LIBS
#    OrocosOCL_DEFINES
#
# This module requires pkg-config

FIND_PACKAGE( OrocosPkgConfig REQUIRED )

IF(OCL_INSTALL)
SET(ENV{PKG_CONFIG_PATH} "${OCL_INSTALL}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
ENDIF(OCL_INSTALL)

OROCOS_PKGCONFIG( "orocos-ocl-${OROCOS_TARGET} >= 1.4" OrocosOCL_FOUND OrocosOCL_INCLUDE_DIR OrocosOCL_DEFINES OrocosOCL_LINK_DIR OrocosOCL_LIBS )

IF( OrocosOCL_FOUND )
    MESSAGE("   Includes in: ${OrocosOCL_INCLUDE_DIR}")
    MESSAGE("   Libraries in: ${OrocosOCL_LINK_DIR}")
    MESSAGE("   Libraries: ${OrocosOCL_LIBS}")
    MESSAGE("   Defines: ${OrocosOCL_DEFINES}")
ELSE( OrocosOCL_FOUND )
    IF( OrocosOCL_FIND_REQUIRED )
	MESSAGE(FATAL_ERROR "The Orocos Component Library has not been found for the ${OROCOS_TARGET} target")
    ELSEIF(NOT OrocosOCL_FIND_QUIETLY)
	MESSAGE(STATUS "The Orocos Component Library has not been found for the ${OROCOS_TARGET} target")
    ENDIF (OrocosOCL_FIND_REQUIRED)
ENDIF( OrocosOCL_FOUND )

