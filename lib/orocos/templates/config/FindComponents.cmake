# Locate Orocos-Components install directory

# This module defines
# _INSTALL where to find include, lib, bin, etc.
# OCL_FOUND, is set to true

FIND_PACKAGE( PkgConfig REQUIRED )

SET(ENV{PKG_CONFIG_PATH} "${OCL_INSTALL}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
PKGCONFIG( "orocos-ocl-${OROCOS_TARGET} >= 1.4" OCL_FOUND OCL_INCLUDE_DIR OCL_DEFINES OCL_LINK_DIR OCL_LIBS )

IF( OCL_FOUND )
    MESSAGE("   Includes in: ${OCL_INCLUDE_DIR}")
    MESSAGE("   Libraries in: ${OCL_LINK_DIR}")
    MESSAGE("   Libraries: ${OCL_LIBS}")
    MESSAGE("   Defines: ${OCL_DEFINES}")

    INCLUDE_DIRECTORIES( ${OCL_INCLUDE_DIR} )
    LINK_DIRECTORIES( ${OCL_LINK_DIR} )
ENDIF ( OCL_FOUND )

