#include "<%= task.name %>.hpp"

using namespace <%= task.component.name %>;

<% task.methods.each do |meth| %>
<%= meth.signature.gsub('(', " #{task.name}::#{meth.method_name}(") %> {
}
<% end %>
