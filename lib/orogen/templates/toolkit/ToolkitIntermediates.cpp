#include <<%= component.name %>ToolkitIntermediates.hpp>
#include <memory>

<%
# We first handle the definitions that declare convertions functions
# (i.e. the ones for which we don't need to generate anything)
toolkit.opaques.
    find_all(&:code_generator).
    each do |opdef|
        type = opdef.type
        intermediate_type = component.find_type(opdef.intermediate)
    %>
    <%= opdef.code_generator.call(type, intermediate_type) %>
<% end %>


<%
# Generate the body of the const-function for from_intermediate,
# if the type does not need a copy.
# See the ToolkitIntermediates.hpp template for more information
toolkit.opaques.
    find_all { |opdef| !opdef.needs_copy? }.
    each do |opdef|
        type = opdef.type
        intermediate_type = component.find_type(opdef.intermediate)
    %>
void <%= component.name %>::from_intermediate(<%= type.ref_type %> value, <%= intermediate_type.arg_type %> _intermediate)
{
    std::auto_ptr< <%= intermediate_type.cxx_name %> > intermediate(new <%= intermediate_type.cxx_name %>);
<%= toolkit.code_fromIntermediate(intermediate_type, false, "    ") %>
}
<%
    end
%>

<%
# Then create the functions that convert a type that contains
# opaques (but is not opaque itself) into its corresponding _m type
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
void <%= component.name %>::from_intermediate(<%= type.ref_type %> value, <%= m_type.arg_type %> intermediate)
{
<%=
        result = ""
        type.from_intermediate(toolkit, result, "    ")
        result %>
}
<% end %>

