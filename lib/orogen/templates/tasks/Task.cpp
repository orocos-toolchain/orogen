#include "<%= task.basename %>.hpp"

using namespace <%= task.component.name %>;

<%= task.basename %>::<%= task.basename %>(std::string const& name)
    : <%= task.basename %>Base(name) {}

<% task.methods.each do |meth| %>
<%= meth.signature.gsub('(', " #{task.basename}::#{meth.method_name}(") %> {
}
<% end %>

<% task.commands.each do |cmd| %>
bool <%= task.basename %>::<%= cmd.work_method_name %><%= cmd.work_signature %> {
    return true;
}

bool <%= task.basename %>::<%= cmd.completion_method_name %><%= cmd.completion_signature %> {
    return true;
}
<% end %>

<% if task.implements?("RTT::FileDescriptorActivity::Provider") %>
/** This method is called after the configuration step by the
 * FileDescriptorActivity to get the file descriptor
 */
int <%= task.basename %>::getFileDescriptor() const
{
    return -1;
}
<% end %>

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See <%= task.basename %>.hpp for more detailed
// documentation about them.

// bool <%= task.basename %>::configureHook() { return true; }
// bool <%= task.basename %>::startHook() { return true; }
<% if task.event_ports.empty? %>
// void <%= task.basename %>::updateHook() {}
<% else %>
// void <%= task.basename %>::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
// {}
<% end %>
// void <%= task.basename %>::errorHook() {}
// void <%= task.basename %>::stopHook() {}
// void <%= task.basename %>::cleanupHook() {}

