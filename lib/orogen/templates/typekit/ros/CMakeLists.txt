set(PROJECT_NAME base_msgs)
set(base_msgs_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
<% typekit_deps = typekit.plugin('ros').dependencies(typekit); %>

# Requires CMAKE_PREFIX_PATH to be set properly
<% %w{catkin genmsg genpy gencpp genlisp}.each do |ros_pkg| %>
find_package(<%= ros_pkg %>)
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
end.compact
%>

<% msg_deps.each do |dep_name| %>
find_package(<%= dep_name %>_msg)
<% end %>

<% if msg_deps.empty? %>
generate_messages()
<% else %>
generate_messages(DEPENDENCIES <%= msg_deps.join(" ") %>)
<% end %>
