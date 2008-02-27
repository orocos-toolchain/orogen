# Locate BFL install directory

# This module defines
# BLF_INSTALL where to find include, lib, bin, etc.
# BLF_FOUND, is set to true

FIND_PACKAGE( PkgConfig REQUIRED )

MESSAGE( STATUS "Detecting BFL" )
MESSAGE( "Looking for BFL in: ${BFL_INSTALL}")
PKGCONFIG( "libbfl-dev >= 0.4.2" BFL_FOUND BFL_INCLUDE_DIRS BFL_DEFINES BFL_LINK_DIRS BFL_LIBS )

IF( BFL_FOUND )
    MESSAGE("   Includes in: ${BFL_INCLUDE_DIRS}")
    MESSAGE("   Libraries in: ${BFL_LINK_DIRS}")
    MESSAGE("   Libraries: ${BFL_LIBS}")
    MESSAGE("   Defines: ${BFL_DEFINES}")

    INCLUDE_DIRECTORIES( ${BFL_INCLUDE_DIRS} )
    LINK_DIRECTORIES( ${BFL_LINK_DIRS} )
ENDIF ( BFL_FOUND )

