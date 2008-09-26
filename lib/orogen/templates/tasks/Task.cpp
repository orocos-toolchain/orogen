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



