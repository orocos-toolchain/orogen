#ifndef <%= component.name %>_INTERMEDIATES_HH
#define <%= component.name %>_INTERMEDIATES_HH

#include <boost/cstdint.hpp>
#include <<%= component.name %>ToolkitTypes.hpp>
<% if toolkit.has_opaques_with_templates? %>
#include <<%= component.name %>ToolkitUser.hpp>
<% end %>

namespace <%= component.name %>
{
    <% toolkit.opaques.find_all { |op| !op.generate_templates? }.each do |opaque_def|
        from = opaque_def.type
        into = component.find_type(opaque_def.intermediate)
        if opaque_def.needs_copy? %>
    void to_intermediate(<%= into.ref_type %> intermediate, <%= from.arg_type %> real_type);
    void from_intermediate(<%= from.ref_type %> real_type, <%= into.ref_type %> intermediate);
        <% else %>
    <%= into.arg_type %> to_intermediate(<%= from.arg_type %> real_type);
    bool from_intermediate(<%= from.ref_type %> real_type, <%= into.cxx_name %>* intermediate);
        <% end %>
    <% end %>

<%
opaques.find_all { |opdef| !opdef.needs_copy? }.each do |opdef|
    type = opdef.type
    intermediate_type = component.find_type(opdef.intermediate)
%>
    void from_intermediate(<%= type.ref_type %> value, <%= intermediate_type.arg_type %> _intermediate);
<% end %>

<%
generated_types.
    find_all { |t| t.contains_opaques? && !t.opaque? }.
    each do |type|
        m_type = toolkit.find_type(type.name + "_m")
        if !m_type
            raise RuntimeError, "no intermediate marshalling type for #{type.name}"
        end
%>
    void to_intermediate(<%= m_type.ref_type %> intermediate, <%= type.arg_type %> value);
    void from_intermediate(<%= type.ref_type %> value, <%= m_type.ref_type %> intermediate);
<% end %>
}

#endif

