#ifndef <%= component.name %>_INTERMEDIATES_HH
#define <%= component.name %>_INTERMEDIATES_HH

#include <boost/cstdint.hpp>
#include <<%= component.name %>ToolkitTypes.hpp>
<% if toolkit.has_opaques_with_templates? %>
#include <<%= component.name %>ToolkitUser.hpp>
<% end %>

namespace <%= component.name %>
{
<%
toolkit.opaques.find_all { |op| !op.generate_templates? }.each do |opaque_def|
    from = opaque_def.type
    into = component.find_type(opaque_def.intermediate)

    if opaque_def.needs_copy? %>
    /** Converts \c real_type into \c intermediate */
    void to_intermediate(<%= into.ref_type %> intermediate, <%= from.arg_type %> real_type);
    /** Converts \c intermediate into \c real_type */
    void from_intermediate(<%= from.ref_type %> real_type, <%= into.arg_type %> intermediate);
    <%
    else
    %>
    /** Returns the intermediate value that is contained in \c real_type */
    <%= into.arg_type %> to_intermediate(<%= from.arg_type %> real_type);
    /** Stores \c intermediate into \c real_type. \c intermediate is owned by \c
     * real_type afterwards. */
    bool from_intermediate(<%= from.ref_type %> real_type, <%= into.cxx_name %>* intermediate);
    <%
    end
end
%>

<%
# Types that do not need copies require the intermediate to be a non-const
# pointer. In case only a const-pointer is available, it is needed to create a
# copy first, and then assign it to the real type (usually, a smart pointer).
#
# This generates the overloaded from_intermediate function that does just that.
opaques.find_all { |opdef| !opdef.needs_copy? }.each do |opdef|
    type = opdef.type
    intermediate_type = component.find_type(opdef.intermediate)
%>
    /** Overloaded from_intermediate function that creates a copy before
     * assigning it to \c value
     */
    void from_intermediate(<%= type.ref_type %> value, <%= intermediate_type.arg_type %> _intermediate);
<% end %>

<%
# This handles the types that contain opaques, but are not opaque themselves. For those,
# orogen generates so-called m-types, that are a mapping from the original (opaque-containing)
# type into a one where opaque fields have been converted to their intermediate counterparts
generated_types.
    find_all { |t| t.contains_opaques? && !t.opaque? }.
    each do |type|
        m_type = toolkit.find_type(type.name + "_m")
        if !m_type
            raise RuntimeError, "no intermediate marshalling type for #{type.name}"
        end
%>
    void to_intermediate(<%= m_type.ref_type %> intermediate, <%= type.arg_type %> value);
    void from_intermediate(<%= type.ref_type %> value, <%= m_type.arg_type %> intermediate);
<% end %>
}

#endif

