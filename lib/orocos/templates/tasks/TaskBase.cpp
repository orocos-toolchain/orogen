#include "tasks/<%= task.name %>.hpp"

using namespace <%= component.name %>;

<%= task.name %>Base::<%= task.name %>Base()
    : RTT::TaskContext("<%= task.name %>")
    , activity(<%= task.rtt_scheduler %>, <%= task.rtt_priority %>, <% if task.period %><%= task.period %>, <% end %>engine())
    <% task.properties.each do |prop| %>
    , _<%= prop.name %>("<%= prop.name %>", "<%= prop.doc %>")
    <% end %>
{
}

