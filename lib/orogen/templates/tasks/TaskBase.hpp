#ifndef <%= component.name.upcase %>_<%= task.basename.upcase %>_TASK_BASE_HPP
#define <%= component.name.upcase %>_<%= task.basename.upcase %>_TASK_BASE_HPP

#include <string>
#include <boost/cstdint.hpp>
#include <<%= task.superclass.header_file %>>
<% if !task.new_methods.empty? || task.superclass.name == "RTT::TaskContext" then %>#include <rtt/Method.hpp><% end %>
<% unless task.new_commands.empty? %>#include <rtt/Command.hpp><% end %>
<% unless task.self_ports.empty? %>#include <rtt/Ports.hpp><% end %>


<% if task.extended_state_support? %>
#include <<%= component.name %>/<%= component.name %>TaskStates.hpp>
<% end %>


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
	<% unless task.self_properties.empty? %>/** Properties */<% end %>
    <% task.self_properties.each do |prop| %>
	RTT::Property< <%= prop.type.cxx_name %> > _<%= prop.name %>;
    <% end %>

	<% unless task.self_ports.empty? %>/** Ports */<% end %>
    <% task.self_ports.each do |port| %>
	<%= port.orocos_class %>< <%= port.type.cxx_name %> > _<%= port.name %>;
    <% end %>

	<% unless task.self_methods.empty? %>/** Methods */<% end %>
    <% task.new_methods.each do |meth| %>
	RTT::Method< <%= meth.signature(false) %> > _<%= meth.name %>;
	virtual <%= meth.signature(true) %> = 0;
    <% end %>

    <% if task.superclass.name == "RTT::TaskContext" %>
	RTT::Method< std::string() > _getModelName;
    <% end %>

	<% unless task.self_commands.empty? %>/** Commands */<% end %>
    <% task.new_commands.each do |cmd| %>
	RTT::Command< <%= cmd.work_signature(false) %> > _<%= cmd.name %>;
	virtual <%= cmd.work_signature %> = 0;
	virtual <%= cmd.completion_signature %> = 0;
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

        bool start();

        <% if task.extended_state_support? && !task.superclass.extended_state_support? %>
        // Reimplement TaskCore base methods to export the states to the outside
        // world
        bool configure();
        bool activate();
        void warning();
        void recovered();
        void error();
        void fatal();
        bool stop();
        bool cleanup();
        bool resetError();
        <% end %>

        virtual std::string getModelName() const;

        <% if task.extended_state_support? %>
        void state(States state);
        void error(States state);
        void fatal(States state);
        States state() const;
        <% end %>
    };
}

#endif

