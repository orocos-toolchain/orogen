# Generated from orogen/lib/orogen/templates/typekit/typelib/CMakeLists.txt

set(libname <%= typekit.name %>-transport-typelib-${OROCOS_TARGET})
rosbuild_add_library(${libname}
    <%= impl.join("\n    ") %>
    TypelibMarshallerBase.cpp)

target_link_libraries(${libname}
    ${TYPELIB_LIBRARIES})
target_link_libraries(${libname}
    LINK_INTERFACE_LIBRARIES)

