# Generated from orogen/lib/orogen/templates/typekit/typelib/CMakeLists.txt

pkg_check_modules(TYPELIB REQUIRED typelib)

set(TYPEKIT_REGISTRY ${CMAKE_INSTALL_PREFIX}/share/orogen/<%= typekit.name %>.tlb)
add_definitions(-DTYPEKIT_REGISTRY="${TYPEKIT_REGISTRY}")
set(libname_typelib <%= typekit.name %>-transport-typelib-${OROCOS_TARGET})
add_library(${libname_typelib} SHARED
    <%= impl.join("\n    ") %>
    TypelibMarshallerBase.cpp)

target_link_libraries(${libname_typelib}
    ${TYPELIB_LIBRARIES}
    ${OrocosRTT_LIBRARIES}
    <% if !typekit.opaques.empty? %>
    <%= typekit.name %>-typekit-${OROCOS_TARGET}
    <% end %>)

target_link_libraries(${libname_typelib}
    LINK_INTERFACE_LIBRARIES)

SET(PKG_CONFIG_FILE_TYPELIB ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>-transport-typelib-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(<%= typekit.name %>-transport-typelib.pc.in ${PKG_CONFIG_FILE_TYPELIB} @ONLY)

install(FILES ${PKG_CONFIG_FILE_TYPELIB} DESTINATION lib/pkgconfig)
install(FILES
    TransportPlugin.hpp
    DESTINATION include/orocos/<%= typekit.name %>/transports/typelib)
install(TARGETS ${libname_typelib} LIBRARY DESTINATION lib/orocos/types)

