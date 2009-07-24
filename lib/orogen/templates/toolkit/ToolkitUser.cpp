#include "<%= component.name %>ToolkitUser.hpp"

<% toolkit.opaques.find_all { |op| op.generate_templates? }.each do |opaque_def|
    from = opaque_def.type
    into = component.find_type(opaque_def.intermediate)
if opaque_def.needs_copy? %>
void <%= component.name %>::to_intermediate(<%= into.cxx_name %>& intermediate, <%= from.cxx_name %> const& real_type);
{
}
void <%= component.name %>::from_intermediate(<%= from.cxx_name %>& real_type, <%= into.cxx_name %>& intermediate)
{
}
<% else %>
<%= into.cxx_name %> const& <%= component.name %>::to_intermediate(<%= from.cxx_name %> const& real_type)
{
}
bool <%= component.name %>::from_intermediate(<%= from.cxx_name %>& real_type, <%= into.cxx_name %>* intermediate)
{
    // Return false if the toolkit should delete +intermediate+ and true if this
    // function took ownership of it.
    return false;
}
<% end %>
<% end %>

