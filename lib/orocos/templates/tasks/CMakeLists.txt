<% task_files = []
   component.tasks.each do |task| 
     if !task_files.empty?
	 task_files << "\n    "
     end
     task_files << "${CMAKE_SOURCE_DIR}/#{Generation::AUTOMATIC_AREA_NAME}/tasks/#{task.name}Base.cpp"
     task_files << "#{task.name}.cpp"
   end
%>
ADD_LIBRARY(<%= component.name %>-tasks-${OROCOS_TARGET} SHARED
    <%= task_files.join(" ") %>)

SET(libname <%= component.name %>-tasks-${OROCOS_TARGET})
INSTALL(TARGETS ${libname}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

