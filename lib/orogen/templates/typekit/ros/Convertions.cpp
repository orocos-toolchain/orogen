/* Generated from orogen/lib/orogen/templates/typekit/ros/Convertions.cpp */

#include "Convertions.hpp"
#include <memory>

<% if typekit.has_opaques? %>
#include "OpaqueConvertions.hpp"
<% end %>

<% typekit.used_typekits.each do |tk| %>
<% next if tk.virtual? %>
#include <<%= tk.name %>/transports/ros/Convertions.hpp>
<% if tk.has_opaques? %>
#include <<%= tk.name %>/typekit/OpaqueConvertions.hpp>
<% end %>
<% end %>

<% typesets.converted_types.each do |type|
    next if ros_base_type?(type) %>
bool ros_integration::toROS( <%= ros_ref_type(type) %> ros, <%= type.arg_type %> value )
{
<%= result = ""
	type.to_ros(typekit, result, " " * 4)
	result 
	%>
    return true;
}
bool ros_integration::fromROS( <%= type.ref_type %> value, <%= ros_arg_type(type) %> ros )
{
<%= result = ""
	type.from_ros(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>
<% typesets.array_types.each do |type| %>
bool ros_integration::toROS( <%= ros_ref_type(type) %> ros, <%= type.arg_type %> value, int length )
{
<%= result = ""
	type.to_ros(typekit, result, " " * 4)
	result 
	%>
    return true;
}
bool ros_integration::fromROS( <%= type.ref_type %> value, int length, <%= ros_arg_type(type) %> ros )
{
<%= result = ""
	type.from_ros(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>

