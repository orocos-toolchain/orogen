#include "Convertions.hpp"
#include <memory>

<% if typekit.has_opaques? %>
#include "OpaqueConvertions.hpp"
<% end %>

<% typekit.used_typekits.each do |tk| %>
#include <<%= tk.name %>/transports/corba/Convertions.hpp>
<% if tk.has_opaques? %>
#include <<%= tk.name %>/OpaqueConvertions.hpp>
<% end %>
<% end %>

<% typesets.converted_types.each do |type| %>
bool orogen_typekits::toCORBA( <%= type.corba_ref_type %> corba, <%= type.arg_type %> value )
{
<%= result = ""
	type.to_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
bool orogen_typekits::fromCORBA( <%= type.ref_type %> value, <%= type.corba_arg_type %> corba )
{
<%= result = ""
	type.from_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>
<% typesets.array_types.each do |type| %>
bool orogen_typekits::toCORBA( <%= type.corba_ref_type %> corba, <%= type.arg_type %> value, int length )
{
<%= result = ""
	type.to_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
bool orogen_typekits::fromCORBA( <%= type.ref_type %> value, int length, <%= type.corba_arg_type %> corba )
{
<%= result = ""
	type.from_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>

<% typesets.opaque_types.each do |opdef|
    type = opdef.type
    intermediate_type = typekit.find_type(opdef.intermediate)
    %>
bool orogen_typekits::toCORBA( <%= intermediate_type.corba_ref_type %> corba, <%= type.arg_type %> value )
{
<%= typekit.code_toIntermediate(intermediate_type, opdef.needs_copy?, "    ") %>
    return toCORBA(corba, intermediate);
}
bool orogen_typekits::fromCORBA( <%= type.ref_type %> value, <%= intermediate_type.corba_arg_type %> corba )
{
    <% if opdef.needs_copy? %>
    <%= intermediate_type.cxx_name %> intermediate;
    if (!fromCORBA(intermediate, corba))
        return false;
<%= typekit.code_fromIntermediate(intermediate_type, true, "    ") %>
    <% else %>
    std::auto_ptr< <%= intermediate_type.cxx_name %> > intermediate(new <%= intermediate_type.cxx_name %>);
    if (!fromCORBA(*intermediate, corba))
        return false;
<%= typekit.code_fromIntermediate(intermediate_type, false, "    ") %>
    <% end %>
    return true;
}
<% end %>

