/* Generated from orogen/lib/orogen/templates/typekit/ros/Convertions.cpp */

#include "Convertions.hpp"
#include <memory>

<% if typekit.has_opaques? %>
#include <<%= typekit.name %>/typekit/OpaqueConvertions.hpp>
<% end %>

<% typekit.used_typekits.each do |tk| %>
<% next if tk.virtual? %>
#include <<%= tk.name %>/transports/ros/Convertions.hpp>
<% if tk.has_opaques? %>
#include <<%= tk.name %>/typekit/OpaqueConvertions.hpp>
<% end %>
<% end %>

namespace ros_convertions {
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
void ros_convertions::toROS( <%= ros_ref_type(ros_type) %> ros, <%= type.arg_type %> value )
{
<%= result = ""
	type.to_ros(typekit, result, " " * 4)
	result 
	%>
}
void ros_convertions::fromROS( <%= type.ref_type %> value, <%= ros_arg_type(ros_type) %> ros )
{
<%= result = ""
	type.from_ros(typekit, result, " " * 4)
	result 
	%>
}
<% end %>

<% convert_array_types.each do |type, ros_type| %>
<%   next if type <= Typelib::NumericType  %>
void ros_convertions::toROS( std::vector< <%= ros_cxx_type(ros_type) %> >& ros, <%= type.cxx_name%> const* value, int length )
{
    ros.resize(length);
    for (int idx = 0; idx < length; ++idx)
        <%= type.call_to_ros("ros[idx]", "value[idx]") %>;
}
void ros_convertions::fromROS( <%= type.cxx_name %>* value, std::vector< <%= ros_cxx_type(ros_type) %> > const& ros, int length )
{
    for (int idx = 0; idx < length; ++idx)
        <%= type.call_from_ros("value[idx]", "ros[idx]") %>;
}
<% end %>

<% convert_boxed_types.each do |type, ros_type| %>
void ros_convertions::toROS( <%= ros_ref_type(ros_type, false) %> ros, <%= type.arg_type %> value )
{ return toROS(ros.data, value); }
void ros_convertions::fromROS( <%= type.ref_type %> value, <%= ros_arg_type(ros_type, false) %> ros )
{ return fromROS(value, ros.data); }
<% end %>

