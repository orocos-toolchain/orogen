/* Generated from orogen/lib/orogen/templates/typekit/ros/ROSConvertions.hpp */
#pragma once

#include <<%= typekit.name %>/typekit/Types.hpp>

<% user_converted_types.each do |type, ros_type| %>
#include <<%= type_to_msg[ros_type.name] %>.h>
<% end %>

namespace ros_convertions {
    /** Converted types: */
    <% user_converted_types.each do |type, ros_type| %>
    void toROS( <%= ros_ref_type(ros_type) %> ros, <%= type.arg_type %> value );
    void fromROS( <%= type.ref_type %> value, <%= ros_arg_type(ros_type) %> ros );
    <% end %>
}

