/* Generated from orogen/lib/orogen/templates/typekit/OpaqueConvertions.cpp */

#include "OpaqueConvertions.hpp"
#include <memory>

<%
# We first handle the definitions that declare convertions functions
# (i.e. the ones for which we don't need to generate anything)
type_sets.opaque_types.
    find_all(&:code_generator).
    each do |opdef|
        type = opdef.type
        intermediate_type = typekit.find_type(opdef.intermediate)
    %>
    <%= opdef.code_generator.call(type, intermediate_type) %>
<% end %>


<%
# Generate the body of the const-function for from_intermediate,
# if the type does not need a copy.
# See the Opaques.hpp template for more information
type_sets.opaque_types.
    find_all { |opdef| !opdef.needs_copy? }.
    each do |opdef|
        type = opdef.type
        intermediate_type = typekit.find_type(opdef.intermediate)
    %>
void <%= typekit.name %>::from_intermediate(<%= type.ref_type %> value, <%= intermediate_type.arg_type %> _intermediate)
{
    std::auto_ptr< <%= intermediate_type.cxx_name %> > intermediate(new <%= intermediate_type.cxx_name %>(_intermediate));
<%= typekit.code_fromIntermediate(intermediate_type, false, "    ") %>
}
<%
    end
%>

<%
# Then create the functions that convert a type that contains
# opaques (but is not opaque itself) into its corresponding _m type
type_sets.types.
    find_all { |t| t.contains_opaques? && !t.opaque? }.
    each do |type|
        m_type = typekit.find_type(type.name + "_m") %>
void <%= typekit.name %>::to_intermediate(<%= m_type.ref_type %> intermediate, <%= type.arg_type %> value)
{
<%=
        result = ""
        type.to_intermediate(typekit, result, "    ")
        result %>
}
void <%= typekit.name %>::from_intermediate(<%= type.ref_type %> value, <%= m_type.arg_type %> intermediate)
{
<%=
        result = ""
        type.from_intermediate(typekit, result, "    ")
        result %>
}
<% end %>

