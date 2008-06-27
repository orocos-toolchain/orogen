## FindPkgConfig.cmake
## by  Albert Strasheim <http://students . ee . sun . ac . za/~albert/>
## and Alex Brooks (a.brooks at acfr . usyd . edu . au)
##
## This module finds packages using pkg-config, which retrieves
## information about packages from special metadata files.
##
## See http://www . freedesktop . org/Software/pkgconfig/
##
## -------------------------------------------------------------------
##
## Usage:
##
## INCLUDE( ${CMAKE_ROOT}/Modules/FindPkgConfig.cmake)
## 
## IF ( OROCOS_PKGCONFIG_EXECUTABLE )
##
##     # Find all the librtk stuff with pkg-config
##     OROCOS_PKGCONFIG( "librtk >= 2.0" HAVE_RTK RTK_INCLUDE_DIRS RTK_DEFINES RTK_LINK_DIRS RTK_LIBS )
##
## ELSE  ( OROCOS_PKGCONFIG_EXECUTABLE )
##
##     # Can't find pkg-config -- have to find librtk somehow else
##
## ENDIF ( OROCOS_PKGCONFIG_EXECUTABLE )
##
##
## Notes:
## 
## You can set the PKG_CONFIG_PATH environment variable to tell
## pkg-config where to search for .pc files. See pkg-config(1) for
## more information.
##
#
# FIXME: IF(WIN32) pkg-config --msvc-syntax ENDIF(WIN32) ???
#
# FIXME: Parsing of pkg-config output is specific to gnu-style flags
#

INCLUDE(FindPkgConfig)
SET(OROCOS_PKGCONFIG_EXECUTABLE ${PKG_CONFIG_EXECUTABLE})
MARK_AS_ADVANCED(OROCOS_PKGCONFIG_EXECUTABLE)

MACRO(OROCOS_PKGCONFIG LIBRARY FOUND INCLUDE_DIRS DEFINES LINKDIRS LINKLIBS)
  pkg_check_modules(OROCOS_PKGCONFIG_ ${LIBRARY})
  SET(${FOUND} ${OROCOS_PKGCONFIG__FOUND})
  SET(${INCLUDE_DIRS} ${OROCOS_PKGCONFIG__INCLUDE_DIRS})

  SET(${DEFINES} ${OROCOS_PKGCONFIG__CFLAGS_OTHER})
  SET(${LINKDIRS} ${OROCOS_PKGCONFIG__LIBRARY_DIRS})
  SET(${LINKLIBS} ${OROCOS_PKGCONFIG__LIBRARIES})
ENDMACRO(OROCOS_PKGCONFIG)
