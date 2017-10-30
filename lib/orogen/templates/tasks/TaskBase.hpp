/* Generated from orogen/lib/orogen/templates/tasks/TaskBase.hpp */

#ifndef <%= task.full_namespace.tr(":","_").upcase %>_<%= task.basename.upcase %>_TASK_BASE_HPP
#define <%= task.full_namespace.tr(":","_").upcase %>_<%= task.basename.upcase %>_TASK_BASE_HPP

#include <string>
#include <boost/cstdint.hpp>
#include <<%= task.superclass.header_file %>>

<% if !task.new_operations.empty? || task.superclass.name == "RTT::TaskContext" %>
#include <rtt/Operation.hpp>
<% end %>
<% if !task.self_ports.empty? %>
#include <rtt/Port.hpp>
<% end %>
<% if task.extended_state_support? %>
#include <<%= project.typekit.name %>/TaskStates.hpp>
<% end %>

<% task.self_properties.sort_by(&:name).each do |p| %>
<%   type = p.type %>
<%=  project.typekit.cxx_gen_includes(*project.typekit.include_for_type(type)) %>
<% if !project.win32? %>
extern template class RTT::Property< <%= type.cxx_name %> >;
<% end %>
<% end %>

<% task.self_attributes.sort_by(&:name).each do |a| %>
<%   type = a.type %>
<%=  project.typekit.cxx_gen_includes(*project.typekit.include_for_type(type)) %>
<% if !project.win32? %>
extern template class RTT::Attribute< <%= type.cxx_name %> >;
<% end %>
<% end %>

<% task.self_ports.sort_by(&:name).each do |p| %>
<%   type = p.type %>
<%=  project.typekit.cxx_gen_includes(*project.typekit.include_for_type(type)) %>
<% if !project.win32? %>
extern template class <%= p.orocos_class %>< <%= type.cxx_name %> >;
extern template class RTT::base::ChannelElement< <%= type.cxx_name %> >;
<% end %>
<% end %>

<% types = task.self_dynamic_ports.
        map { |p| [p.orocos_class, p.type] if p.type }.
        compact %>
<% types.each do |orocos_class, type| %>
<%=    project.typekit.cxx_gen_includes(*project.typekit.include_for_type(type)) %>
<% if !project.win32? %>
extern template class <%= orocos_class %>< <%= type.cxx_name %> >;
extern template class RTT::base::ChannelElement< <%= type.cxx_name %> >;
<% end %>
<% end %>

<% task.self_operations.sort_by(&:name).each do |op| %>
<%    op.used_types.each do |type| %>
<%=       project.typekit.cxx_gen_includes(*project.typekit.include_for_type(type)) %>
<%    end %>
<% end %>

<% task.implemented_classes.sort.each do |class_name, include_file| %>
#include <<%= include_file %>> // to get <%= class_name %>
<% end %>

<% code_before, code_after =
    task.base_header_code.partition(&:first)
   code_before.map! { |_, c| c.call }
   code_after.map! { |_, c| c.call }
%>

<%= code_before.sort.join("\n") %>

<% task.full_namespace.split("::").each do |space| %>
namespace <%= space %>{
<% end %>
    class <%= task.basename %>;
    /** The oroGen-generated part of <%= task.name %>
     *
     * It is used by oroGen and its plugins to define the task interfaces and set
     * up the component. Never edit yourself, as its content would be
     * overwritten at the next code generation.
     */
    class <%= task.basename %>Base : public ::<%= task.superclass.name %>
        <% task.implemented_classes.each do |class_name, _| %>
        , public <%= class_name %>
        <% end %>
    {
    protected:
        // Common implementation of interface setup for both task constructors
        void setupComponentInterface();

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
	<%= task.basename %>Base(std::string const& name, RTT::ExecutionEngine* engine<%= ", TaskCore::TaskState initial_state" unless task.fixed_initial_state? %>);
        ~<%= task.basename %>Base();

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
        void report(States state);
        void state(States state);
        void error(States state);
        void fatal(States state);
        void exception(States state);
        States state() const;
        <% end %>

<% task.base_hook_code.keys.sort.each do |hook_name| %>
<%    snippets = task.base_hook_code[hook_name] %>
<%    next if snippets.empty? %>
<%    is_boolean = (hook_name == "start" || hook_name == "configure") %>
        <%= (is_boolean ? 'bool' : 'void') %> <%= hook_name %>Hook();
<% end %>
    };

<% task.full_namespace.split("::").each do |space| %>
}
<% end %>

<%= code_after.sort.join("\n") %>

#endif

