/* Generated from orogen/lib/orogen/templates/typekit/ros/Convertions.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_ROS_CONVERTIONS_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_ROS_CONVERTIONS_HPP

#include "Types.hpp"
#include <boost/cstdint.hpp>
#include <string>

<% typesets.converted_types.each do |type|
    if type < Typelib::CompoundType %>
#include <<%= ros_message_name(type, true) %>.h>
<%  end %>
<% end %>

namespace ros_integration {
    /** Converted types: */
    <% typesets.converted_types.each do |type|
        next if type.name == "/std/string" %>
    bool toROS( <%= ros_ref_type(type) %> ros, <%= type.arg_type %> value );
    bool fromROS( <%= type.ref_type %> value, <%= ros_arg_type(type) %> ros );
    <% end %>
    /** Array types: */
    <% typesets.array_types.each do |type| %>
    bool toROS( <%= ros_ref_type(type) %> ros, <%= type.arg_type %> value, int length );
    bool fromROS( <%= type.ref_type %> value, int length, <%= ros_arg_type(type) %> ros );
    <% end %>
}

#endif


