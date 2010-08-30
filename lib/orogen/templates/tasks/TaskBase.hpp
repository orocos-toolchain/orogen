/* Generated from orogen/lib/orogen/templates/tasks/TaskBase.hpp */

#ifndef <%= component.name.upcase %>_<%= task.basename.upcase %>_TASK_BASE_HPP
#define <%= component.name.upcase %>_<%= task.basename.upcase %>_TASK_BASE_HPP

#include <string>
#include <boost/cstdint.hpp>
#include <<%= task.superclass.header_file %>>
<% if !task.new_operations.empty? || task.superclass.name == "RTT::TaskContext" then %>#include <rtt/Operation.hpp><% end %>
<% unless task.self_ports.empty? %>#include <rtt/Port.hpp><% end %>


<% if task.extended_state_support? %>
#include <<%= component.typekit.name %>/<%= component.name %>TaskStates.hpp>
<% end %>


<% task.used_typekits.each do |tk| %>
#include <<%= tk.name %>/Types.hpp>
<% end %>
<% task.implemented_classes.each do |class_name, include_file| %>
#include <<%= include_file %>> // to get <%= class_name %>
<% end %>
<% if component.typekit %>#include "<%= component.typekit.name %>/Types.hpp"<% end %>


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

	<% unless task.self_operations.empty? %>/** Operations */<% end %>
    <% task.new_operations.each do |op| %>
	RTT::Operation< <%= op.signature(false) %> > _<%= op.name %>;
        // If you get the following error:
        //   Cannot instanciate an object of class <%= task.name %> because the following methods are abstract:
        //     <%= op.signature(true) %>
        //
        // it means that you did not implement the "<%= op.method_name %>" method
        // in <%= task.name %>, which is required for the <%= op.name %>
        // operation
        //
        // Please update tasks/<%= task.basename %>.*
        //
        // See
        //   templates/tasks/<%= task.basename %>.hpp and
        //   templates/tasks/<%= task.basename %>.cpp
        // For template definitions
	virtual <%= op.signature(true) %> = 0;
    <% end %>

    <% if task.superclass.name == "RTT::TaskContext" %>
	RTT::Operation< std::string() > _getModelName;
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
        bool recover();
        bool stop();
        bool cleanup();
        void error();
        void fatal();
        void exception();
        <% end %>

        virtual std::string getModelName() const;

        <% if task.extended_state_support? %>
        void state(States state);
        void error(States state);
        void fatal(States state);
        void exception(States state);
        States state() const;
        <% end %>
    };
}

#endif

