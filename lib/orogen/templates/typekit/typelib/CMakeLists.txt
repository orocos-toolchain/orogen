pkg_check_modules(TYPELIB REQUIRED typelib)

set(libname <%= typekit.name %>-transport-typelib-${OROCOS_TARGET})
add_library(${libname} SHARED
    <%= impl.join("\n    ") %>
    TypelibMarshallerBase.cpp)

target_link_libraries(${libname}
    ${TYPELIB_LIBRARIES})
target_link_libraries(${libname}
    LINK_INTERFACE_LIBRARIES)

install(TARGETS ${libname} LIBRARY DESTINATION lib/orocos/types)

