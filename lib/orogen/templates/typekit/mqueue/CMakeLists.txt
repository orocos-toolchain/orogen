# Generated from orogen/lib/orogen/templates/typekit/mqueue/CMakeLists.txt

find_package(Orocos-RTT)

# Set up generation of IDL files for MQueue support
IF (NOT OROCOS-RTT_MQUEUE_FOUND)
    MESSAGE(FATAL_ERROR "could not find development files for Orocos/MQueue")
ENDIF(NOT OROCOS-RTT_MQUEUE_FOUND)

set(MQueue_FILES "")

# Generate the MQueue transport
set(libname_mqueue <%= typekit.name %>-transport-mqueue-${OROCOS_TARGET})
add_library(${libname_mqueue} SHARED
    <%= relatives = []
        impl.each do |file| 
	    relatives << File.basename(file)
        end
        relatives.sort.join("\n    ") %>
    ${MQueue_FILES})
target_link_libraries(${libname_mqueue}
    <%= typekit.name %>-typekit-${OROCOS_TARGET}
    ${OROCOS-RTT_MQUEUE_LIBRARIES})

<%= typekit_deps = typekit.plugin('mqueue').dependencies(typekit);
    Generation.cmake_pkgconfig_link('mqueue', '${libname_mqueue}', typekit_deps) %>
target_link_libraries(${libname_mqueue} LINK_INTERFACE_LIBRARIES ${OROCOS-RTT_MQUEUE_LIBRARIES})

SET(PKG_CONFIG_FILE_MQueue ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>-transport-mqueue-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(<%= typekit.name %>-transport-mqueue.pc.in ${PKG_CONFIG_FILE_MQueue} @ONLY)

install(TARGETS ${libname_mqueue} LIBRARY DESTINATION ${TYPEKIT_LIB_INSTALL_DIR})
install(FILES ${PKG_CONFIG_FILE_MQueue} DESTINATION lib/pkgconfig)
install(FILES
    <%= relatives = []
        headers.each do |file| 
	    relatives << File.basename(file)
        end
        relatives.sort.join("\n    ") %>
    DESTINATION include/orocos/<%= typekit.name %>/transports/mqueue)

