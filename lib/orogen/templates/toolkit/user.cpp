#include "<%= component.name %>ToolkitUser.hpp"

<% toolkit.marshal_as.each do |from, (into, _)|
    into = component.find_type(into) %>
void <%= component.name %>::to_intermediate(<%= into.cxx_name %>& intermediate, <%= from.cxx_name %> const& real_type)
{
}
void <%= component.name %>::from_intermediate(<%= from.cxx_name %>& real_type, <%= into.cxx_name %> const& intermediate)
{
}
<% end %>

