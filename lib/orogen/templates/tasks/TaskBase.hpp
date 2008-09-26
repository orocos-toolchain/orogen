#ifndef <%= component.name.upcase %>_<%= task.basename.upcase %>_TASK_BASE_HPP
#define <%= component.name.upcase %>_<%= task.basename.upcase %>_TASK_BASE_HPP

#include <string>
#include <<%= task.superclass.header_file %>>
<% component.used_toolkits.each do |name, _| %>
#include <toolkit/<%= name %>ToolkitTypes.hpp>
<% end %>
<% if component.toolkit %>#include "toolkit/<%= component.name %>ToolkitTypes.hpp"<% end %>

<% unless task.methods.empty? %>#include <rtt/Method.hpp><% end %>
<% unless task.commands.empty? %>#include <rtt/Command.hpp><% end %>
<% unless task.ports.empty? %>#include <rtt/Ports.hpp><% end %>

namespace <%= component.name %> {
    class <%= task.basename %>;
    class <%= task.basename %>Base : public <%= task.superclass.name %>
    {
    protected:
	<%= task.basename %>& _self;

	<% unless task.properties.empty? %>/** Properties */<% end %>
    <% task.properties.each do |prop| %>
	RTT::Property< <%= prop.type.cxx_name %> > _<%= prop.name %>;
    <% end %>

	<% unless task.ports.empty? %>/** Ports */<% end %>
    <% task.ports.each do |port| %>
	RTT::<%= port.orocos_class %>< <%= port.type.cxx_name %> > _<%= port.name %>;
    <% end %>

	<% unless task.methods.empty? %>/** Methods */<% end %>
    <% task.methods.each do |meth| %>
	RTT::Method< <%= meth.signature(false) %> > _<%= meth.name %>;
    <% end %>

	<% unless task.commands.empty? %>/** Commands */<% end %>
    <% task.commands.each do |cmd| %>
	RTT::Command< bool<%= cmd.work_signature(false) %> > _<%= cmd.name %>;
    <% end %>

    public:
	<%= task.basename %>Base(std::string const& name);
    };
}

#endif

