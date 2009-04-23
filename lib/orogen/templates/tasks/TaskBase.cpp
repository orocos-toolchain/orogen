<% if task.self_methods.empty? && task.self_commands.empty? %>
#include "tasks/<%= task.basename %>Base.hpp"
<% else %>
#include "tasks/<%= task.basename %>.hpp"
<% end %>

using namespace <%= component.name %>;

<%= task.basename %>Base::<%= task.basename %>Base(std::string const& name<%= ", TaskCore::TaskState state" unless task.fixed_initial_state? %>)
<% if task.superclass.fixed_initial_state? %>
    : <%= task.superclass.name %>(name)
<% elsif task.needs_configuration? %>
    : <%= task.superclass.name %>(name, TaskCore::PreOperational)
<% else %>
    : <%= task.superclass.name %>(name, state)
<% end %>
<% unless task.self_methods.empty? && task.self_commands.empty? %>
    , _self(static_cast<<%= task.basename %>&>(*this))
<% end %>
    <% task.self_properties.each do |prop| %>
    , _<%= prop.name %>("<%= prop.name %>", "<%= prop.doc %>")<% end %>
    <% task.self_ports.each do |port| %>
    , _<%= port.name %>("<%= port.name %>")<% end %>
    <% task.self_methods.each do |meth| %>
    // If you get the following error:
    //   <%= meth.method_name %> is not a member of <%= task.name %>
    // it means that you did not implement the "<%= meth.method_name %>" method
    // in <%= task.name %>. Please update tasks/<%= task.basename %>.*
    // See
    //   templates/tasks/<%= task.basename %>.hpp and
    //   templates/tasks/<%= task.basename %>.cpp
    , _<%= meth.name %>("<%= meth.name %>", &<%= task.name %>::<%= meth.method_name %>, &_self)<% end %>
    <% task.self_commands.each do |cmd| %>
    // If you get one of the following errors:
    //   <%= cmd.work_method_name %> is not a member of <%= task.name %>
    //   <%= cmd.completion_method_name %> is not a member of <%= task.name %>
    // it means that you did not implement one of those two methods in in
    // <%= task.name %>. Please update tasks/<%= task.basename %>.*
    // See
    //   templates/tasks/<%= task.basename %>.hpp and
    //   templates/tasks/<%= task.basename %>.cpp
    , _<%= cmd.name %>("<%= cmd.name %>", &<%= task.name %>::<%= cmd.work_method_name %>, &<%= task.name %>::<%= cmd.completion_method_name %>, &_self)<% end %>
{
    <% task.self_properties.each do |prop|
        if prop.default_value %>
        _<%= prop.name %>.set(<%= prop.default_value.inspect %>);
        <% end %>
    properties()->addProperty( &_<%= prop.name %> );<% end %>
    <% (task.self_ports - task.event_ports).each do |port| %>
    ports()->addPort( &_<%= port.name %>, "<%= port.doc %>" );<% end %>
    <% (task.event_ports & task.self_ports).each do |port| %>
    ports()->addEventPort( &_<%= port.name %>, std::string("<%= port.doc %>") );<% end %>
    <% (task.self_methods + task.self_commands).each do |callable| 
	argument_setup = callable.arguments.
	    map { |n, _, d| ", \"#{n}\", \"#{d}\"" }.
	    join("")
	kind = callable.class.name.gsub(/^.*::/, '')
    %>
    <%= kind.downcase %>s()->add<%= kind %>( &_<%= callable.name %>, "<%= callable.doc %>"<%= argument_setup %>);<% end %>
}

