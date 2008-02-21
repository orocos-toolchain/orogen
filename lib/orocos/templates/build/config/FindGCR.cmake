# Locate GCR install directory

# This module defines
# BLF_INSTALL where to find include, lib, bin, etc.
# BLF_FOUND, is set to true

FIND_PACKAGE( PkgConfig REQUIRED )

SET(ENV{PKG_CONFIG_PATH} "${GCR_INSTALL}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
PKGCONFIG( "gcr >= 1.0" GCR_FOUND GCR_INCLUDE_DIRS GCR_DEFINES GCR_LINK_DIRS GCR_LIBS )

IF( GCR_FOUND )
    MESSAGE("   Includes in: ${GCR_INCLUDE_DIRS}")
    MESSAGE("   Libraries in: ${GCR_LINK_DIRS}")
    MESSAGE("   Libraries: ${GCR_LIBS}")
    MESSAGE("   Defines: ${GCR_DEFINES}")

    INCLUDE_DIRECTORIES( ${GCR_INCLUDE_DIRS} )
    LINK_DIRECTORIES( ${GCR_LINK_DIRS} )
ENDIF ( GCR_FOUND )

