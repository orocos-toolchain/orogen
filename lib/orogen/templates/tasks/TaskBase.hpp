#ifndef <%= component.name.upcase %>_<%= task.basename.upcase %>_TASK_BASE_HPP
#define <%= component.name.upcase %>_<%= task.basename.upcase %>_TASK_BASE_HPP

#include <string>
#include <<%= task.superclass.header_file %>>
<% task.used_toolkits.each do |name, _| %>
#include <toolkit/<%= name %>ToolkitTypes.hpp>
<% end %>
<% task.implemented_classes.each do |class_name, include_file| %>
#include <<%= include_file %>> // to get <%= class_name %>
<% end %>
<% if component.toolkit %>#include "toolkit/<%= component.name %>ToolkitTypes.hpp"<% end %>

<% unless task.self_methods.empty? %>#include <rtt/Method.hpp><% end %>
<% unless task.self_commands.empty? %>#include <rtt/Command.hpp><% end %>
<% unless task.self_ports.empty? %>#include <rtt/Ports.hpp><% end %>

namespace <%= component.name %> {
    class <%= task.basename %>;
    class <%= task.basename %>Base : public <%= task.superclass.name %>
        <% task.implemented_classes.each do |class_name, _| %>
        , public <%= class_name %>
        <% end %>
    {
    protected:
    <% unless task.self_methods.empty? && task.self_commands.empty? %>
	<%= task.basename %>& _self;
    <% end %>

	<% unless task.self_properties.empty? %>/** Properties */<% end %>
    <% task.self_properties.each do |prop| %>
	RTT::Property< <%= prop.type.cxx_name %> > _<%= prop.name %>;
    <% end %>

	<% unless task.self_ports.empty? %>/** Ports */<% end %>
    <% task.self_ports.each do |port| %>
	<%= port.orocos_class %>< <%= port.type.cxx_name %> > _<%= port.name %>;
    <% end %>

	<% unless task.self_methods.empty? %>/** Methods */<% end %>
    <% task.self_methods.each do |meth| %>
	RTT::Method< <%= meth.signature(false) %> > _<%= meth.name %>;
    <% end %>

	<% unless task.self_commands.empty? %>/** Commands */<% end %>
    <% task.self_commands.each do |cmd| %>
	RTT::Command< bool<%= cmd.work_signature(false) %> > _<%= cmd.name %>;
    <% end %>

    public:
	<%= task.basename %>Base(std::string const& name<%= ", TaskCore::TaskState initial_state" unless task.fixed_initial_state? %>);
    };
}

#endif

