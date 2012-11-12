/* Generated from orogen/lib/orogen/templates/typekit/ros/Convertions.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_ROS_CONVERTIONS_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_ROS_CONVERTIONS_HPP

#include "Types.hpp"
#include <boost/cstdint.hpp>
#include <string>

<% all_messages.each do |msg_name| %>
#include <<%= typekit.name %>_msgs/<%= msg_name %>.h>
<% end %>

namespace ros_integration {
    /** Converted types: */
    <% convert_types.each do |type, ros_type| %>
    void toROS( <%= ros_ref_type(ros_type) %> ros, <%= type.arg_type %> value );
    void fromROS( <%= type.ref_type %> value, <%= ros_arg_type(ros_type) %> ros );
    <% end %>
    /** Array types: */
    <% convert_array_types.each do |type, ros_type| %>
    void toROS( <%= ros_typename(ros_type) %>* ros, <%= type.cxx_name%> const* value, int length );
    void fromROS( <%= type.cxx_name %>* value, <%= ros_typename(ros_type) %> const* ros, int length );
    <% end %>
}

#endif


