#include <rtt/Component.hpp>

<% deployable_tasks = component.self_tasks.find_all { |t| !t.abstract? } %>
<% deployable_tasks.each do |task| %>
#include <<%= component.name.downcase %>/<%= task.basename %>.hpp>
<% end %>

ORO_CREATE_COMPONENT_TYPE();
<% deployable_tasks.each do |task| %>
ORO_LIST_COMPONENT_TYPE( <%= task.name %> );
<% end %>

