# Generated from orogen/lib/orogen/templates/typekit/typelib/CMakeLists.txt

pkg_check_modules(TYPELIB REQUIRED typelib)
include_directories(${TYPELIB_INCLUDE_DIRS})
link_directories(${TYPELIB_LIBRARY_DIRS})

set(libname <%= typekit.name %>-transport-typelib-${OROCOS_TARGET})
add_library(${libname} SHARED
    <%= impl.join("\n    ") %>
    TypelibMarshallerBase.cpp)

target_link_libraries(${libname}
    ${TYPELIB_LIBRARIES})
target_link_libraries(${libname}
    LINK_INTERFACE_LIBRARIES)

SET(PKG_CONFIG_FILE_TYPELIB ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>-transport-typelib-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(<%= typekit.name %>-transport-typelib.pc.in ${PKG_CONFIG_FILE_TYPELIB} @ONLY)

install(FILES ${PKG_CONFIG_FILE_TYPELIB} DESTINATION lib/pkgconfig)
install(TARGETS ${libname} LIBRARY DESTINATION lib/orocos/types)

