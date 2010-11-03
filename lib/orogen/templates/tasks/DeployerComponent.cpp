#include <ocl/Component.hpp>
<% component.self_tasks.each do |task| %>
#include <<%= component.name.downcase %>/<%= task.basename %>.hpp>
<% end %>

ORO_CREATE_COMPONENT_TYPE();
<% component.self_tasks.each do |task| %>
ORO_LIST_COMPONENT_TYPE( <%= task.name %> );
<% end %>

