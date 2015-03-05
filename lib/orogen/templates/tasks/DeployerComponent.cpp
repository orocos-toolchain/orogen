#include <rtt/Component.hpp>

<% deployable_tasks = project.self_tasks.find_all { |t| !t.abstract? } %>
<% deployable_tasks.each do |task| %>
#include <<%= project.name.downcase %>/<%= task.basename %>.hpp>
<% end %>

ORO_CREATE_COMPONENT_TYPE();
<% deployable_tasks.each do |task| %>
ORO_LIST_COMPONENT_TYPE( <%= task.name %> );
<% end %>

namespace orogen
{
<% deployable_tasks.each do |task| %>
    RTT::TaskContext* create_<%= task.name.gsub(/[^\w]/, '_') %>(std::string const& instance_name)
    {
        return new <%= task.name %>(instance_name);
    }
<% end %>
}
