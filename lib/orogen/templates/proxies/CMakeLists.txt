# Generated from orogen/lib/orogen/templates/proxies/CMakeLists.txt


SET(PROXYLIB_NAME <%= component.name %>-proxies)

add_definitions(-DRTT_COMPONENT)

<% 
    pkg_conf_deps = ["orocos_cpp_base"]
    pkg_conf_deps.concat(dependencies(false))
%>

<% pkg_conf_deps.each do |dependency| %>
pkg_check_modules(<%= dependency.upcase%> REQUIRED <%= dependency%>)
<% end %>

include(<%= component.name %>TaskLib)

ADD_LIBRARY(${PROXYLIB_NAME} SHARED 
<%= 
   task_files = String.new
   component.self_tasks.each do |task| 
     task_files << "    #{task.basename}.cpp\n"
   end
   task_files
%>)
<% if component.typekit %>
add_dependencies(${PROXYLIB_NAME}
    regen-typekit)
<% end %>

TARGET_LINK_LIBRARIES(${PROXYLIB_NAME}
    ${OROCOS_CPP_LIBRARIES}
    ${OrocosRTT_LIBRARIES}
<% pkg_conf_deps.each do |dependency| %>
    ${<%=dependency.upcase%>_LIBRARIES}
<% end %>
<% if component.typekit %>
    <%= component.name %>-typekit-${OROCOS_TARGET}
<% component.enabled_transports.each do |transport_name| %>
    <%= component.name %>-transport-<%= transport_name %>-${OROCOS_TARGET}
<% end %>
<% end %>
)

SET_TARGET_PROPERTIES(${PROXYLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${<%= component.name.upcase %>_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${PROXYLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

    
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROXYLIB_NAME}.pc.in
            ${CMAKE_CURRENT_BINARY_DIR}/${PROXYLIB_NAME}.pc @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROXYLIB_NAME}.pc
            DESTINATION lib/pkgconfig)

INSTALL(FILES 
<%= 
   include_files = String.new
   component.self_tasks.each do |task| 
     include_files << "    #{task.basename}.hpp\n"
   end
   include_files
%>
    DESTINATION include/orocos/<%= component.name.downcase %>/proxies/)

