#include "<%= task.basename %>.hpp"
<% if task.default_activity
      activity_type = Orocos::Generation::ACTIVITY_TYPES[task.default_activity.first]
%>
#include <rtt/<%= activity_type %>.hpp>
<% end %>

using namespace <%= task.component.name %>;

<% if task.default_activity
    activity_type = Orocos::Generation::ACTIVITY_TYPES[task.default_activity.first]
%>
RTT::<%= activity_type %>* <%= task.basename %>::get<%= activity_type %>()
{ return dynamic_cast< RTT::<%= activity_type %>* >(getActivity().get()); }
<% end %>

<%= task.basename %>::<%= task.basename %>(std::string const& name<%= ", TaskCore::TaskState initial_state" unless task.fixed_initial_state? %>)
    : <%= task.basename %>Base(name<%= ", initial_state" unless task.fixed_initial_state? %>)
{
}

<% task.self_methods.each do |meth| %>
<%= meth.signature { "#{task.basename}::#{meth.method_name}" } %>
{
}
<% end %>

<% task.self_commands.each do |cmd| %>
<%= cmd.work_signature { "#{task.basename}::#{cmd.work_method_name}" } %>
{
    return true;
}

<%= cmd.completion_signature { "#{task.basename}::#{cmd.completion_method_name}" } %>
{
    return true;
}
<% end %>

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See <%= task.basename %>.hpp for more detailed
// documentation about them.

// bool <%= task.basename %>::configureHook()
// {
//     return true;
// }
// bool <%= task.basename %>::startHook()
// {
//     return true;
// }
<% if task.event_ports.empty? %>
// void <%= task.basename %>::updateHook()
// {
// }
<% else %>
// void <%= task.basename %>::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
// {
// }
<% end %>
// void <%= task.basename %>::errorHook()
// {
// }
// void <%= task.basename %>::stopHook()
// {
// }
// void <%= task.basename %>::cleanupHook()
// {
// }

