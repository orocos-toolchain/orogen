ADD_EXECUTABLE(<%= component.name %> ${CMAKE_SOURCE_DIR}/<%= Generation::AUTOMATIC_AREA_NAME %>/main.cpp)
TARGET_LINK_LIBRARIES(<%= component.name %> ${OROCOS_COMPONENT_LIBRARIES})

<% component.used_toolkits.each do |name| %>
    TARGET_LINK_LIBRARIES(<%= component.name %> ${<%= name %>_TOOLKIT_LIBS})
<% end %>
<% if !component.tasks.empty? %>
TARGET_LINK_LIBRARIES(<%= component.name %> <%= component.name %>-tasks-${OROCOS_TARGET})
<% end %>
INSTALL(TARGETS <%= component.name %>
    RUNTIME DESTINATION bin)
<% if component.corba_enabled? %>
    TARGET_LINK_LIBRARIES(<%= component.name %> ${OrocosCORBA_LIBS} TAO_Strategies)
<% end %>

