/* Generated from orogen/lib/orogen/templates/typekit/OpaqueConvertions.cpp */

#include <<%= typekit.name %>/typekit/OpaqueConvertions.hpp>
#include <<%= typekit.name %>/typekit/OpaqueTypes.hpp>
#include <memory>
#include <stdexcept>

<% opaque_types             = type_sets.opaque_types.find_all { |op| !op.generate_templates? } %>
<% types_containing_opaques = type_sets.types.find_all { |t| t.contains_opaques? && !t.opaque? } %>

<% needed_typekits = Set.new
   types_containing_opaques.each do |t|
       t.direct_dependencies.each do |dep_t|
           if dep_t.contains_opaques?
               needed_typekits |= typekit.imported_typekits_for(dep_t).to_set
           end
        end
   end
%>
<% needed_typekits.sort_by(&:name).each do |tk| %>
#include <<%= tk.name %>/typekit/OpaqueConvertions.hpp>
<% end %>

<% # We first handle the definitions that declare convertions functions
# (i.e. the ones for which we don't need to generate anything)
type_sets.opaque_types.find_all(&:code_generator).
    each do |opdef|
        type = opdef.type
        target_type = typekit.intermediate_type_for(type)
    %>
    <%= opdef.code_generator.call(type, target_type) %>
<% end %>


<%
# Generate the body of the const-function for from_intermediate,
# if the type does not need a copy.
# See the Opaques.hpp template for more information
type_sets.opaque_types.find_all { |opdef| !opdef.needs_copy? }.
    each do |opdef|
        type = opdef.type
        target_type = typekit.intermediate_type_for(type)
    %>
void orogen_typekits::fromIntermediate(<%= type.ref_type %> value, <%= target_type.arg_type %> _intermediate)
{
    std::auto_ptr< <%= target_type.cxx_name %> > intermediate(new <%= target_type.cxx_name %>(_intermediate));
<%= typekit.code_fromIntermediate(target_type, false, "    ") %>
}
<%
    end
%>

<%
# Then create the functions that convert a type that contains
# opaques (but is not opaque itself) into its corresponding _m type
types_containing_opaques.each do |type|
        m_type = intermediate_type_for(type) %>
void orogen_typekits::toIntermediate(<%= m_type.ref_type %> intermediate, <%= type.arg_type %> value<%= ", int length" if type < Typelib::ArrayType %>)
{
<%=
        result = ""
        type.to_intermediate(typekit, result, "    ")
        result %>
}
void orogen_typekits::fromIntermediate(<%= type.ref_type %> value<%= ", int length" if type < Typelib::ArrayType %>, <%= m_type.arg_type %> intermediate)
{
<%=
        result = ""
        type.from_intermediate(typekit, result, "    ")
        result %>
}
<% end %>

