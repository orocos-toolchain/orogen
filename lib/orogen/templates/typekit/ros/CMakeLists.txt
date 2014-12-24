<% ros_pkg_name = typekit.plugin('ros').ros_package_name_for_typekit(typekit) %>
set(PROJECT_NAME <%= ros_pkg_name %>)
set(<%= ros_pkg_name %>_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
<% typekit_deps = typekit.plugin('ros').dependencies %>

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
    # Don't depend on transports that actually don't define any messages
    pkg_rosmap = TypekitMarshallers::ROS.load_rosmap_by_package_name(tk.name)
    next if pkg_rosmap.empty?
    "orogen_#{tk.name}_msgs"
end.compact.to_set
user_converted_types.each do |type, ros_type|
    msg = type_to_msg[ros_type.name]
    msg_deps << msg.split('/').first
end
msg_deps = msg_deps.to_a.sort
%>

<% msg_deps.each do |dep_name| %>
find_package(<%= dep_name %>)
<% end %>

# This is a shameless copy of code from share/genmsg/extras/genmsg-extras.cmake
# genmsg-groovy forces us to find_package the transitive dependencies from our
# direct dependencies.

macro(find_genmsg_transitive_packages)
  set(workspaces ${CATKIN_WORKSPACES})
  list(FIND workspaces ${CATKIN_DEVEL_PREFIX} _index)
  if(_index EQUAL -1)
    list(INSERT workspaces 0 ${CATKIN_DEVEL_PREFIX})
  endif()
  
  set(pending_deps ${ARGN})
  set(handled_deps "")
  while(pending_deps)
    list(GET pending_deps 0 dep)
    list(REMOVE_AT pending_deps 0)
    list(APPEND handled_deps ${dep})
  
    find_package(${dep} REQUIRED)
  
    unset(config CACHE)
    set(filename "share/${dep}/cmake/${dep}-msg-paths.cmake")
    find_file(config ${filename} PATHS ${workspaces}
      NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
    if("${config}" STREQUAL "config-NOTFOUND")
      message(FATAL_ERROR "Could not find '${filename}' (searched in '${workspaces}').")
    endif()
    include(${config})
    unset(config CACHE)
  
    # add transitive msg dependencies
    foreach(recdep ${${dep}_MSG_DEPENDENCIES})
      set(all_deps ${handled_deps} ${pending_deps})
      list(FIND all_deps ${recdep} _index)
      if(_index EQUAL -1)
        list(APPEND pending_deps ${recdep})
      endif()
    endforeach()
  endwhile()
endmacro()

<% if msg_deps.empty? %>
generate_messages()
<% else %>
find_genmsg_transitive_packages("<%= msg_deps.join("\" \"") %>")
generate_messages(DEPENDENCIES <%= msg_deps.join(" ") %>)
<% end %>
include_directories(BEFORE ${CMAKE_BINARY_DIR}/${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/)
<% end # end of if !all_messages.empty?
%>

set(libname_ros <%= typekit.name %>-transport-ros-${OROCOS_TARGET})
add_library(${libname_ros} SHARED
    "<%= impl.join("\"\n    \"") %>")
<% deps = all_messages.map { |msg_name| "${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/${PROJECT_NAME}/#{msg_name}.h" }.join(";") %>
set_source_files_properties(<%= impl.join(" ") %>
    PROPERTIES OBJECT_DEPENDS "<%= deps %>")
target_link_libraries(${libname_ros}
    <%= typekit.name %>-typekit-${OROCOS_TARGET}
    ${OrocosROS_LIBRARIES})
<%= Generation.cmake_pkgconfig_link('ros', '${libname_ros}', typekit_deps) %>

SET(PKG_CONFIG_FILE_ROS ${CMAKE_CURRENT_BINARY_DIR}/<%= typekit.name %>-transport-ros-${OROCOS_TARGET}.pc)
configure_file(<%= typekit.name %>-transport-ros.pc.in ${PKG_CONFIG_FILE_ROS} @ONLY)

# Needed to generate the config.cmake file
configure_file(<%= ros_pkg_name %>-config.cmake.in <%= ros_pkg_name %>-config.cmake @ONLY)
install(TARGETS ${libname_ros} LIBRARY DESTINATION lib/orocos/types)
install(FILES ${PKG_CONFIG_FILE_ROS} DESTINATION lib/pkgconfig)
install(FILES <%= rosmap %> DESTINATION share/orogen/)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/<%= ros_pkg_name %>-config.cmake DESTINATION share/<%= ros_pkg_name %>/cmake)
install(FILES
    <%= headers.join("\n   ") %>
    Convertions.hpp
    DESTINATION include/orocos/<%= typekit.name %>/transports/ros)
