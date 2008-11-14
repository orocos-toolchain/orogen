#include "<%= component.name %>ToolkitUser.hpp"

<% toolkit.marshal_as.each do |from, (into, _)|
    into = component.find_type(into) %>
void <%= component.name %>UserMarshalling::dump_<%= from.method_name %>(<%= from.cxx_name %> const& from, <%= into.cxx_name %>& into)
{
}
void <%= component.name %>UserMarshalling::load_<%= from.method_name %>(<%= from.cxx_name %>& from, <%= into.cxx_name %> const& into)
{
}
<% end %>

