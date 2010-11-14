/* Generated from orogen/lib/orogen/templates/typekit/corba/Convertions.cpp */

#include "Convertions.hpp"
#include <memory>

<% if typekit.has_opaques? %>
#include "OpaqueConvertions.hpp"
<% end %>

<% typekit.used_typekits.each do |tk| %>
#include <<%= tk.name %>/transports/corba/Convertions.hpp>
<% if tk.has_opaques? %>
#include <<%= tk.name %>/typekit/OpaqueConvertions.hpp>
<% end %>
<% end %>

<% typesets.converted_types.each do |type|
    target_type = typekit.intermediate_type_for(type) %>
bool orogen_typekits::toCORBA( <%= target_type.corba_ref_type %> corba, <%= type.arg_type %> value )
{
<%= result = ""
	type.to_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
bool orogen_typekits::fromCORBA( <%= type.ref_type %> value, <%= target_type.corba_arg_type %> corba )
{
<%= result = ""
	type.from_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>
<% typesets.array_types.each do |type|
    target_type = typekit.intermediate_type_for(type) %>
bool orogen_typekits::toCORBA( <%= target_type.corba_ref_type %> corba, <%= type.arg_type %> value, int length )
{
<%= result = ""
	type.to_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
bool orogen_typekits::fromCORBA( <%= type.ref_type %> value, int length, <%= target_type.corba_arg_type %> corba )
{
<%= result = ""
	type.from_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>

