#ifndef <%= component.name.upcase %>_<%= task.basename.upcase %>_TASK_BASE_HPP
#define <%= component.name.upcase %>_<%= task.basename.upcase %>_TASK_BASE_HPP

#include <string>
#include <boost/cstdint.hpp>
#include <<%= task.superclass.header_file %>>
<% unless task.self_methods.empty? %>#include <rtt/Method.hpp><% end %>
<% unless task.self_commands.empty? %>#include <rtt/Command.hpp><% end %>
<% unless task.self_ports.empty? %>#include <rtt/Ports.hpp><% end %>

<% task.used_toolkits.each do |tk| %>
#include <toolkit/<%= tk.name %>ToolkitTypes.hpp>
<% end %>
<% task.implemented_classes.each do |class_name, include_file| %>
#include <<%= include_file %>> // to get <%= class_name %>
<% end %>
<% if component.toolkit %>#include "toolkit/<%= component.name %>ToolkitTypes.hpp"<% end %>


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
        <% if extended_state_support? %>
        enum States
        {
            <% states = task.each_state.to_a
               states.each_with_index do |(state_name, state_type), i| %>
                <%= task.state_local_value_name(state_name, state_type) %> = <%= task.state_global_value_name(state_name, state_type) %><%= ',' if i != states.size - 1 %>
            <% end %>
        };
        <% end %>
        
	<%= task.basename %>Base(std::string const& name<%= ", TaskCore::TaskState initial_state" unless task.fixed_initial_state? %>);

        <% if task.extended_state_support? && !task.superclass.extended_state_support? %>
        // Reimplement TaskCore base methods to export the states to the outside
        // world
        bool configure();
        bool activate();
        bool start();
        bool stop();
        bool cleanup();
        bool resetError();
        <% end %>

        <% if task.extended_state_support? %>
        void state(States state);
        void error(States state);
        void fatal(States state);
        <%= task.state_type_name %> state() const;
        <% end %>
    };
}

#endif

