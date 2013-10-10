/* Generated from orogen/lib/orogen/templates/typekit/Opaques.cpp */

#include <<%= typekit.name %>/typekit/OpaqueTypes.hpp>
#include <<%= typekit.name %>/typekit/Opaques.hpp>

    /** Returns the intermediate value that is contained in \c real_type */
    /** Stores \c intermediate into \c real_type. \c intermediate is owned by \c
     * real_type afterwards. */
    /** Release ownership of \c real_type on the corresponding intermediate
     * pointer.
     */

<% type_sets.opaque_types.find_all { |op| op.generate_templates? }.each do |opaque_def|
    from = opaque_def.type
    into = typekit.intermediate_type_for(from)
if opaque_def.needs_copy? %>
void orogen_typekits::toIntermediate(<%= into.ref_type %> intermediate, <%= from.arg_type %> real_type)
{
}

void orogen_typekits::fromIntermediate(<%= from.ref_type %> real_type, <%= into.arg_type %> intermediate)
{
}
<% else %>
<%= into.arg_type %> orogen_typekits::toIntermediate(<%= from.arg_type %> real_type)
{
}
bool orogen_typekits::fromIntermediate(<%= from.ref_type %> real_type, <%= into.cxx_name %>* intermediate)
{
    // Return false if the typekit should delete +intermediate+ and true if this
    // function took ownership of it.
    return false;
}
void orogen_typekits::release(<%= from.ref_type %> real_type)
{
}
<% end %>
<% end %>

