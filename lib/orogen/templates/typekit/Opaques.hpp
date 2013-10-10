/* Generated from orogen/lib/orogen/templates/typekit/Opaques.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name %>_USER_MARSHALLING_HH
#define __OROGEN_GENERATED_<%= typekit.name %>_USER_MARSHALLING_HH

#include <<%= typekit.name %>/typekit/OpaqueFwd.hpp>

namespace orogen_typekits
{
    <% type_sets.opaque_types.find_all { |op| op.generate_templates? }.each do |opaque_def|
        from = opaque_def.type
        into = typekit.find_type(opaque_def.intermediate)
        if opaque_def.needs_copy? %>
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(<%= into.ref_type %> intermediate, <%= from.arg_type %> real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(<%= from.ref_type %> real_type, <%= into.arg_type %> intermediate);
        <% else %>
    /** Returns the intermediate value that is contained in \c real_type */
    <%= into.arg_type %> toIntermediate(<%= from.arg_type %> real_type);
    /** Stores \c intermediate into \c real_type. \c intermediate is owned by \c
     * real_type afterwards. */
    bool fromIntermediate(<%= from.ref_type %> real_type, <%= into.cxx_name %>* intermediate);
    /** Release ownership of \c real_type on the corresponding intermediate
     * pointer.
     */
    void release(<%= from.ref_type %> real_type);
        <% end %>
    <% end %>
}

#endif

