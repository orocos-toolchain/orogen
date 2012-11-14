/* Generated from orogen/lib/orogen/templates/typekit/ros/Convertions.cpp */

#include "ROSConvertions.hpp"

<% user_converted_types.each do |type, ros_type|  %>
void ros_convertions::toROS( <%= ros_ref_type(ros_type) %> ros, <%= type.arg_type %> value )
{
}
void ros_convertions::fromROS( <%= type.ref_type %> value, <%= ros_arg_type(ros_type) %> ros )
{
}
<% end %>
