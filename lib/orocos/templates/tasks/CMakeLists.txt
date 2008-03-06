<% task_files = []
   component.tasks.each do |task| 
     task_files << "${CMAKE_SOURCE_DIR}/#{Generation::AUTOMATIC_AREA_NAME}/tasks/#{task.name}Base.cpp"
     task_files << "#{task.name}.cpp"
   end
%>
ADD_LIBRARY(<%= component.name %>-tasks-${OROCOS_TARGET} <%= task_files.join(" ") %>)

