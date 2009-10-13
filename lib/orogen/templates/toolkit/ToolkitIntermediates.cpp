#include <<%= component.name %>ToolkitIntermediates.hpp>
#include <memory>

<%
# Generate first the automatic convertion functions
opaques.each do |opdef|
    type = opdef.type
    intermediate_type = component.find_type(opdef.intermediate)
    %>
<%= if opdef.code_generator
      opdef.code_generator.call(type, intermediate_type)
    end %>
<% end %>


// Const-functions for opaques that don't require a copy
<%
# Generate first the automatic convertion functions
opaques.find_all { |opdef| !opdef.needs_copy? }.each do |opdef|
    type = opdef.type
    intermediate_type = component.find_type(opdef.intermediate)
%>
void <%= component.name %>::from_intermediate(<%= type.ref_type %> value, <%= intermediate_type.arg_type %> _intermediate)
{
    std::auto_ptr< <%= intermediate_type.cxx_name %> > intermediate(new <%= intermediate_type.cxx_name %>);
<%= toolkit.code_fromIntermediate(intermediate_type, false, "    ") %>
}
<% end %>

<%
# Then create the functions that convert a type that contains
# opaques into its corresponding _m type
generated_types.
    find_all { |t| t.contains_opaques? && !t.opaque? }.
    each do |type|
        m_type = toolkit.find_type(type.name + "_m") %>
void <%= component.name %>::to_intermediate(<%= m_type.ref_type %> intermediate, <%= type.arg_type %> value)
{
<%=
        result = ""
        type.to_intermediate(toolkit, result, "    ")
        result %>
}
void <%= component.name %>::from_intermediate(<%= type.ref_type %> value, <%= m_type.ref_type %> intermediate)
{
<%=
        result = ""
        type.from_intermediate(toolkit, result, "    ")
        result %>
}
<% end %>

