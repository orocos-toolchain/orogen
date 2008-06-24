#include "<%= task.name %>.hpp"

using namespace <%= task.component.name %>;

<%= task.name %>::<%= task.name %>(std::string const& name)
    : <%= task.name %>Base(name) {}

<% task.methods.each do |meth| %>
<%= meth.signature.gsub('(', " #{task.name}::#{meth.method_name}(") %> {
}
<% end %>

<% task.commands.each do |cmd| %>
bool <%= task.name %>::<%= cmd.work_method_name %><%= cmd.work_signature %> {
    return true;
}

bool <%= task.name %>::<%= cmd.completion_method_name %><%= cmd.completion_signature %> {
    return true;
}
<% end %>



