# Generated from orogen/lib/orogen/templates/proxies/CMakeLists.txt


SET(PROXYLIB_NAME <%= component.name %>-proxies)

pkg_check_modules(OROCOS_CPP REQUIRED orocos_cpp)
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
<%= 
    list = component.used_typekits.dup
    list << component.typekit if component.typekit
    result = ""
    list.each do |tk|
        next if tk.name == "rtt"
        next if tk.name == "logger"

        component.enabled_transports.each do |transport_name|
            result << "    #{tk.name}-transport-#{transport_name}-gnulinux \n"
        end
        result << "    #{tk.name}-typekit-gnulinux"
        result << " \n"
    end
    result
%>)
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

