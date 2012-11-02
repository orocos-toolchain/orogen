set(PROJECT_NAME <%= typekit.name %>_msgs)
set(<%= typekit.name %>_msgs_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
<% typekit_deps = typekit.plugin('ros').dependencies(typekit); %>

find_package(catkin REQUIRED)
find_package(genmsg REQUIRED)

<%= Generation.cmake_pkgconfig_require(typekit_deps, 'ros') %>

<% if !all_messages.empty?
%>
add_message_files(DIRECTORY msg
    FILES
    <%= all_messages.join(".msg\n    ") %>.msg)

<%
msg_deps = typekit.used_typekits.map do |tk|
    next if tk.virtual?
    tk.name
end.compact.sort
%>

<% msg_deps.each do |dep_name| %>
find_package(<%= dep_name %>_msgs)
<% end %>

<% if msg_deps.empty? %>
generate_messages()
<% else %>
generate_messages(DEPENDENCIES <%= msg_deps.join("_msgs ") %>_msgs)
<% end %>
include_directories(BEFORE ${CMAKE_BINARY_DIR}/gen/cpp)
<% end # end of if !all_messages.empty?
%>

set(libname_ros <%= typekit.name %>-transport-ros-${OROCOS_TARGET})
add_library(${libname_ros} SHARED
    <%= impl.join("\n    ") %>)
<% deps = all_messages.map { |msg_name| "${CMAKE_BINARY_DIR}/gen/cpp/#{typekit.name}_msgs/#{msg_name}.h" }.join(";") %>
set_source_files_properties(<%= impl.join(" ") %>
    PROPERTIES OBJECT_DEPENDS "<%= deps %>")
target_link_libraries(${libname_ros}
    <%= typekit.name %>-typekit-${OROCOS_TARGET}
    ${OrocosROS_LIBRARIES})
<%= Generation.cmake_pkgconfig_link('ros', '${libname_ros}', typekit_deps) %>

SET(PKG_CONFIG_FILE_ROS ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>-transport-ros-${OROCOS_TARGET}.pc)
configure_file(<%= typekit.name %>-transport-ros.pc.in ${PKG_CONFIG_FILE_ROS} @ONLY)

# Needed to generate the config.cmake file
configure_file(<%= typekit.name %>_msgs-config.cmake.in <%= typekit.name %>_msgs-config.cmake @ONLY)
install(TARGETS ${libname_ros} LIBRARY DESTINATION lib/orocos/types)
install(FILES ${PKG_CONFIG_FILE_ROS} DESTINATION lib/pkgconfig)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>_msgs-config.cmake DESTINATION share/<%= typekit.name %>_msgs/cmake)
install(FILES
    <%= headers.join("\n   ") %>
    Convertions.hpp
    DESTINATION include/orocos/<%= typekit.name %>/transports/ros)
