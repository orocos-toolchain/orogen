#include "<%= component.name %>ToolkitUser.hpp"

<% toolkit.opaques.each do |opaque_def|
    from = opaque_def.type
    into = component.find_type(opaque_def.intermediate) %>
void <%= component.name %>::to_intermediate(<%= into.cxx_name %>& intermediate, <%= from.cxx_name %> const& real_type)
{
}
void <%= component.name %>::from_intermediate(<%= from.cxx_name %>& real_type, <%= into.cxx_name %> const& intermediate)
{
}
<% end %>

