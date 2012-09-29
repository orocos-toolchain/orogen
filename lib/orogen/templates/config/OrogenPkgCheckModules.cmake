include(FindPkgConfig) # This is the Cmake 2.6 FindPkgConfig macro
macro(orogen_pkg_check_modules VARNAME)
    if (NOT ${VARNAME}_FOUND)
        pkg_check_modules(${VARNAME} ${ARGN})
        foreach(${VARNAME}_lib ${${VARNAME}_LIBRARIES})
          set(_${VARNAME}_lib NOTFOUND)
          find_library(_${VARNAME}_lib NAMES ${${VARNAME}_lib} HINTS ${${VARNAME}_LIBRARY_DIRS})
          if (NOT _${VARNAME}_lib)
            set(_${VARNAME}_lib ${${VARNAME}_lib})
          endif()
          list(APPEND _${VARNAME}_LIBRARIES ${_${VARNAME}_lib})
        endforeach()
        list(APPEND _${VARNAME}_LIBRARIES ${${VARNAME}_LDFLAGS_OTHER})
        set(${VARNAME}_LIBRARIES ${_${VARNAME}_LIBRARIES} CACHE INTERNAL "")
    endif()
endmacro()


