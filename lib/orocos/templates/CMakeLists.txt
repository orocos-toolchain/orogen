#CMAKE_MINIMUM_REQUIRED(VERSION 2.2.2)

PROJECT(<%= component.name %>)
SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orocos/config")

INCLUDE(OrocosComponent)

INCLUDE_DIRECTORIES(BEFORE ${CMAKE_SOURCE_DIR}/.orocos)
INCLUDE_DIRECTORIES(BEFORE ${CMAKE_SOURCE_DIR})

<% task_files = []
   component.tasks.each do |task| 
     task_files << "${CMAKE_SOURCE_DIR}/.orocos/tasks/#{task.name}Base.cpp"
     task_files << "tasks/#{task.name}.cpp"
   end
%>
ADD_EXECUTABLE(<%= component.name %> ${CMAKE_SOURCE_DIR}/.orocos/main.cpp <%= task_files.join(" ") %>)
TARGET_LINK_LIBRARIES(<%= component.name %> ${OROCOS_COMPONENT_LIBRARIES})

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

