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

namespace ros_integration {
    template<typename ROS, typename CXX>
    inline void toROS(ROS& ros, CXX const& value) { ros = value; }
    // Cast needed for the enums
    template<typename ROS, typename CXX>
    inline void fromROS(CXX& value, ROS const& ros) { value = static_cast<CXX>(ros); }
    template<typename ROS, typename CXX>
    inline void toROS(ROS& ros, CXX const* value, int length)
    {
        ros.resize(length);
        for (int idx = 0; idx < length; ++idx)
            ros[idx] = value[idx];
    }
    template<typename ROS, typename CXX>
    inline void fromROS(CXX* value, ROS const& ros, int length)
    {
        for (int idx = 0; idx < length; ++idx)
            value[idx] = static_cast<CXX>(ros[idx]);
    }
}

<% convert_types.each do |type, ros_type|  %>
bool ros_integration::toROS( <%= ros_ref_type(ros_type) %> ros, <%= type.arg_type %> value )
{
<%= result = ""
	type.to_ros(typekit, result, " " * 4)
	result 
	%>
    return true;
}
bool ros_integration::fromROS( <%= type.ref_type %> value, <%= ros_arg_type(ros_type) %> ros )
{
<%= result = ""
	type.from_ros(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>

<% convert_array_types.each do |type, ros_type| %>
bool ros_integration::toROS( <%= ros_ref_type(ros_type) %> ros, <%= type.cxx_name %> const* value, int length )
{
<%= result = ""
	type.to_ros(typekit, result, " " * 4)
	result 
	%>
    return true;
}
bool ros_integration::fromROS( <%= type.cxx_name %>* value, <%= ros_arg_type(ros_type) %> ros, int length )
{
<%= result = ""
	type.from_ros(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>

