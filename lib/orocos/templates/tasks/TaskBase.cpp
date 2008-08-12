#include "tasks/<%= task.basename %>.hpp"

using namespace <%= component.name %>;

<%= task.basename %>Base::<%= task.basename %>Base(std::string const& name)
<% if task.fixed_initial_state? %>
    : <%= task.superclass.name %>(name)
<% else %>
    : <%= task.superclass.name %>(name, TaskContext::<%= task.initial_state %>)
<% end %>
    , _self(static_cast<<%= task.basename %>&>(*this))
    <% task.properties.each do |prop| %>
    , _<%= prop.name %>("<%= prop.name %>", "<%= prop.doc %>")<% end %>
    <% task.ports.each do |port| %>
    <% if port.kind_of?(BufferPort) && port.write_access? %>, _<%= port.name %>("<%= port.name %>", <%= port.size %>)
    <% else %>, _<%= port.name %>("<%= port.name %>")<% end %>
    <% end %>
    <% task.methods.each do |meth| %>
    , _<%= meth.name %>("<%= meth.name %>", &<%= task.name %>::<%= meth.method_name %>, &_self)<% end %>
    <% task.commands.each do |cmd| %>
    , _<%= cmd.name %>("<%= cmd.name %>", &<%= task.name %>::<%= cmd.work_method_name %>, &<%= task.name %>::<%= cmd.completion_method_name %>, &_self)<% end %>
{
    <% task.properties.each do |prop|
        if prop.default_value %>
        _<%= prop.name %>.set(<%= prop.default_value.inspect %>);
        <% end %>
    properties()->addProperty( &_<%= prop.name %> );<% end %>
    <% task.ports.each do |port| %>
    ports()->addPort( &_<%= port.name %>, "<%= port.doc %>" );<% end %>
    <% (task.methods + task.commands).each do |callable| 
	argument_setup = callable.arguments.
	    map { |n, _, d| ", \"#{n}\", \"#{d}\"" }.
	    join("")
	kind = callable.class.name.gsub(/^.*::/, '')
    %>
    <%= kind.downcase %>s()->add<%= kind %>( &_<%= callable.name %>, "<%= callable.doc %>"<%= argument_setup %>);<% end %>
}

