set(PROJECT_NAME base_msgs)
set(base_msgs_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
<% typekit_deps = typekit.plugin('ros').dependencies(typekit); %>

# Cheat !
<% %w{catkin genmsg genpy gencpp genlisp}.each do |ros_pkg| %>
orogen_pkg_check_modules(<%= ros_pkg %> REQUIRED <%= ros_pkg %>)
set(<%= ros_pkg %>_DIR ${<%= ros_pkg %>_PREFIX}/share/<%= ros_pkg %>/cmake)
<% end %>

find_package(catkin REQUIRED)
find_package(genmsg REQUIRED)
<%= Generation.cmake_pkgconfig_require(typekit_deps, 'ros') %>

add_message_files(DIRECTORY msg
    FILES
    <%= all_messages.join(".msg\n    ") %>.msg)

<%
msg_deps = typekit.used_typekits.map do |tk|
    next if tk.virtual?
    tk.name
end
%>

<% msg_deps.each do |dep_name| %>
set(<%= dep_name %>_FOUND true)
set(<%= dep_name %>_DIR ${<%= dep_name %>_TRANSPORT_ROS}_DIR/msg)
<% end %>

<% if msg_deps.empty? %>
generate_messages()
<% else %>
generate_messages(DEPENDENCIES <%= msg_deps.join(" ") %>)
<% end %>
