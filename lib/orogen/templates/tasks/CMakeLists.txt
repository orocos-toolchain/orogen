# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(<%= project.name %>TaskLib)
ADD_LIBRARY(${<%= project.name.upcase %>_TASKLIB_NAME} SHARED 
    ${<%= project.name.upcase %>_TASKLIB_SOURCES})
<% if project.typekit %>
add_dependencies(${<%= project.name.upcase %>_TASKLIB_NAME}
    regen-typekit)
<% end %>

TARGET_LINK_LIBRARIES(${<%= project.name.upcase %>_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${<%= project.name.upcase %>_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${<%= project.name.upcase %>_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${<%= project.name.upcase %>_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${<%= project.name.upcase %>_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${<%= project.name.upcase %>_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${<%= project.name.upcase %>_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${<%= project.name.upcase %>_TASKLIB_HEADERS}
    DESTINATION include/orocos/<%= project.name.downcase %>)

