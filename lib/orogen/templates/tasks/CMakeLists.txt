<% 
   include_files = []
   task_files = []
   component.self_tasks.each do |task| 
     if !task_files.empty?
	 task_files << "\n    "
     end
     task_files << "${CMAKE_SOURCE_DIR}/#{Generation::AUTOMATIC_AREA_NAME}/tasks/#{task.basename}Base.cpp"
     task_files << "#{task.basename}.cpp"
     include_files << "${CMAKE_SOURCE_DIR}/#{Generation::AUTOMATIC_AREA_NAME}/tasks/#{task.basename}Base.hpp"
     include_files << "#{task.basename}.hpp"
   end
%>

SET(libname <%= component.name %>-tasks-${OROCOS_TARGET})
ADD_LIBRARY(${libname} SHARED <%= task_files.join(" ") %>)
TARGET_APPEND_LDFLAGS(${libname} "${DEPENDENCIES_LDFLAGS}")

INSTALL(TARGETS ${libname}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES <%= include_files.join(" ") %>
    DESTINATION include/orocos/<%= component.name.downcase %>)
