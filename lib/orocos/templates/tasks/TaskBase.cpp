#include "tasks/<%= task.name %>.hpp"

using namespace <%= component.name %>;

<%= task.name %>Base::<%= task.name %>Base()
    : RTT::TaskContext("<%= task.name %>")
    , _activity(<%= task.rtt_scheduler %>, <%= task.rtt_priority %>, <% if task.period %><%= task.period %>, <% end %>engine())
    , _self(static_cast<<%= task.name %>&>(*this))
    <% task.properties.each do |prop| %>
    , _<%= prop.name %>("<%= prop.name %>", "<%= prop.doc %>")
    <% end %>
    <% task.methods.each do |meth| %>
    , _<%= meth.name %>("<%= meth.name %>", &<%= task.name %>::<%= meth.method_name %>, &_self)
    <% end %>
{
    <% task.properties.each do |prop| %>
    properties()->addProperty( &_<%= prop.name %> );
    <% end %>
    <% task.methods.each do |meth| 
	argument_setup = meth.arguments.
	    map { |n, _, d| ", \"#{n}\", \"#{d}\"" }.
	    join("")
    %>
    methods()->addMethod( &_<%= meth.name %>, "<%= meth.doc %>"<%= argument_setup %>);
    <% end %>
}

