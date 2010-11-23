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
  <% next if tk.virtual? %>
#include <<%= tk.name %>/Types.hpp>
<% end %>
<% task.implemented_classes.each do |class_name, include_file| %>
#include <<%= include_file %>> // to get <%= class_name %>
<% end %>
<% if component.typekit %>#include "<%= component.typekit.name %>/Types.hpp"<% end %>

<% code_before, code_after =
    task.base_header_code.partition(&:first)
   code_before.map! { |_, c| c.call }
   code_after.map! { |_, c| c.call }
%>

<%= code_before.join("\n") %>

namespace <%= component.name %> {
    class <%= task.basename %>;
    class <%= task.basename %>Base : public ::<%= task.superclass.name %>
        <% task.implemented_classes.each do |class_name, _| %>
        , public <%= class_name %>
        <% end %>
    {
    protected:

<%= task.self_base_methods.
    sort_by(&:name).
    map { |m| m.with_indent(8, :declaration) }.
    compact.join("\n") %>
<%= task.self_base_members.
    sort_by { |m| [m.kind, m.name] }.
    map { |m| m.with_indent(8, :declaration) }.
    compact.join("\n") %>

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

        <% if task.extended_state_support? %>
        void state(States state);
        void error(States state);
        void fatal(States state);
        void exception(States state);
        States state() const;
        <% end %>
    };
}

<%= code_after.join("\n") %>

#endif

