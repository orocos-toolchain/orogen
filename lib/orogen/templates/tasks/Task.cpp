/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "<%= task.basename %>.hpp"

using namespace <%= task.project.name %>;

<%= task.basename %>::<%= task.basename %>(std::string const& name<%= ", TaskCore::TaskState initial_state" unless task.fixed_initial_state? %>)
    : <%= task.basename %>Base(name<%= ", initial_state" unless task.fixed_initial_state? %>)
{
}

<%= task.basename %>::<%= task.basename %>(std::string const& name, RTT::ExecutionEngine* engine<%= ", TaskCore::TaskState initial_state" unless task.fixed_initial_state? %>)
    : <%= task.basename %>Base(name, engine<%= ", initial_state" unless task.fixed_initial_state? %>)
{
}

<%= task.basename %>::~<%= task.basename %>()
{
}

<%= task.self_user_methods.sort_by(&:name).
    map { |m| m.with_indent(0, :definition) }.
    compact.join("\n\n") %>

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See <%= task.basename %>.hpp for more detailed
// documentation about them.

<% %w{configure start update error stop cleanup}.each do |hook_name| %>
<%    snippets = task.user_hook_code[hook_name] %>
<%    is_boolean = (hook_name == "start" || hook_name == "configure") %>
<%= (is_boolean ? 'bool' : 'void') %> <%= task.basename %>::<%= hook_name %>Hook()
{
<% if is_boolean %>
    if (! <%= task.basename %>Base::<%= hook_name %>Hook())
        return false;
<% else %>
    <%= task.basename %>Base::<%= hook_name %>Hook();
<% end %>
<% snippets.each do |code| %>
<%     if code.respond_to?(:to_str) %>
<%= code %>
<%     else %>
<%= code.call %>
<%     end %>
<% end %>
<% if is_boolean %>
    return true;
<% end %>
}
<% end %>
