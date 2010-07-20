#include "<%= component.name %>TypekitUser.hpp"

    /** Returns the intermediate value that is contained in \c real_type */
    /** Stores \c intermediate into \c real_type. \c intermediate is owned by \c
     * real_type afterwards. */
    /** Release ownership of \c real_type on the corresponding intermediate
     * pointer.
     */

<% typekit.opaques.find_all { |op| op.generate_templates? }.each do |opaque_def|
    from = opaque_def.type
    into = component.find_type(opaque_def.intermediate)
if opaque_def.needs_copy? %>
void <%= component.name %>::to_intermediate(<%= into.ref_type %> intermediate, <%= from.arg_type %> real_type)
{
}

void <%= component.name %>::from_intermediate(<%= from.ref_type %> real_type, <%= into.arg_type %> intermediate)
{
}
<% else %>
<%= into.arg_type %> <%= component.name %>::to_intermediate(<%= from.arg_type %> real_type)
{
}
bool <%= component.name %>::from_intermediate(<%= from.ref_type %> real_type, <%= into.cxx_name %>* intermediate)
{
    // Return false if the typekit should delete +intermediate+ and true if this
    // function took ownership of it.
    return false;
}
void <%= component.name %>::release(<%= from.ref_type %> real_type)
{
}
<% end %>
<% end %>

