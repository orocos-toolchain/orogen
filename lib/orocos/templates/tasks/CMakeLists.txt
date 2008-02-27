<% task_files = []
   component.tasks.each do |task| 
     task_files << "${CMAKE_CURRENT_SOURCE_DIR}/.orocos/#{task.name}Base.cpp"
     task_files << "#{task.name}.cpp"
   end
%>
ADD_LIBRARY(<%= task_files.join(" ") %>

