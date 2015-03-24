# Generated from orogen/lib/orogen/templates/proxies/CMakeLists.txt


SET(PROXYLIB_NAME <%= project.name %>-proxies)

add_definitions(-DRTT_COMPONENT)

<% 
    pkg_conf_deps = proxy_gen.dependencies(false)
%>
<%= Orocos::Generation.cmake_pkgconfig_require(pkg_conf_deps) %>

include(<%= project.name %>TaskLib)

ADD_LIBRARY(${PROXYLIB_NAME} SHARED 
<%= 
   task_files = String.new
   project.self_tasks.each do |task| 
     task_files << "    #{task.basename}.cpp\n"
   end
   task_files
%>)
<% if project.typekit %>
add_dependencies(${PROXYLIB_NAME}
    regen-typekit)
<% end %>



<%= Orocos::Generation.cmake_pkgconfig_link('core', '${PROXYLIB_NAME}', pkg_conf_deps) %>

SET_TARGET_PROPERTIES(${PROXYLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${<%= project.name.upcase %>_TASKLIB_INTERFACE_LIBRARIES}")

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
   project.self_tasks.each do |task| 
     include_files << "    #{task.basename}.hpp\n"
   end
   include_files
%>
    DESTINATION include/orocos/<%= project.name.downcase %>/proxies/)

