<% task_files = []
   component.tasks.each do |task| 
     next if task.external_definition?
     if !task_files.empty?
	 task_files << "\n    "
     end
     task_files << "${CMAKE_SOURCE_DIR}/#{Generation::AUTOMATIC_AREA_NAME}/tasks/#{task.name}Base.cpp"
     task_files << "#{task.name}.cpp"
   end
%>

SET(libname <%= component.name %>-tasks-${OROCOS_TARGET})
ADD_LIBRARY(${libname} SHARED <%= task_files.join(" ") %>)
TARGET_APPEND_LDFLAGS(${libname} "${DEPENDENCIES_LDFLAGS}")

INSTALL(TARGETS ${libname}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

