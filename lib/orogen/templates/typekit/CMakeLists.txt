# Generated from orogen/lib/orogen/templates/typekit/CMakeLists.txt

##
# Header for typegen-generated typekits
#
<% if typekit.standalone? %>
project("<%= typekit.name %>-typekit")
cmake_minimum_required(VERSION 2.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
set(ROS_BUILD_TYPE Debug)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/install/lib/orocos/types)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/config")

# In Orogen components, the build target is specified at generation time
set(OROCOS_TARGET "<%= typekit.orocos_target %>")

ADD_CUSTOM_TARGET(regen
    <% ruby_bin   = RbConfig::CONFIG['RUBY_INSTALL_NAME']
       typegen_bin = File.expand_path('../bin/typegen', Orocos::Generation.base_dir) %>
    <%= ruby_bin %> <%= typegen_bin %> <%= Orocos::Generation.command_line_options.join(" ") %>
    WORKING_DIRECTORY <%= Orocos::Generation.generation_directory %>)

<% end %>
# END if typekit.standalone?
##

include_directories(BEFORE ${CMAKE_CURRENT_BINARY_DIR})
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/types)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${OrocosRTT_INCLUDE_DIRS})
add_definitions(${OrocosRTT_CFLAGS_OTHER})
link_directories(${OrocosRTT_LIBRARY_DIRS})

<% if typekit.has_opaques_with_templates? %>
include_directories(${CMAKE_SOURCE_DIR}/typekit)
<% end %>

# Now set up the dependencies
<%= typekit_deps = typekit.dependencies
    Generation.cmake_pkgconfig_require(typekit_deps) %>
<%  typekit_deps.each do |dep_def|
        if dep_def.in_context?('core', 'include') %>
set(PKG_CFLAGS ${PKG_CFLAGS} ${<%= dep_def.var_name %>_CFLAGS})
        <% end %>
    <% end %>
string(REPLACE ";" "\" \"" PKG_CFLAGS "\"${PKG_CFLAGS}\"")

# Generate the base typekit shared library
set(libname <%= typekit.name %>-typekit-${OROCOS_TARGET})
rosbuild_add_library(${libname}
    <%= implementation_files.sort.join("\n  ") %>
    ${TYPEKIT_ADDITIONAL_SOURCES}
    ${TOOLKIT_ADDITIONAL_SOURCES}
    <% if !typekit.opaques.empty? %>
        <% if typekit.has_opaques_with_templates? %>
            ${PROJECT_SOURCE_DIR}/typekit/Opaques.cpp
        <% end %>
        OpaqueConvertions.cpp
    <% end %>)

<%= Generation.cmake_pkgconfig_link_noncorba('${libname}', typekit_deps) %>
target_link_libraries(${libname} ${OrocosRTT_LIBRARIES} ${TYPEKIT_ADDITIONAL_LIBRARIES} ${TOOLKIT_ADDITIONAL_LIBRARIES})
target_link_libraries(${libname} LINK_INTERFACE_LIBRARIES ${OrocosRTT_LIBRARIES})

<% typekit.each_plugin do |plg|
    if plg.separate_cmake? %>
    add_subdirectory(transports/<%= plg.name %>)
    <% end
end %>
