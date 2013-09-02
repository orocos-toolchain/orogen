/* Generated from orogen/lib/orogen/templates/typekit/ros/Convertions.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_ROS_CONVERTIONS_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_ROS_CONVERTIONS_HPP

#include "Types.hpp"
#include <boost/cstdint.hpp>
#include <string>

<% if !user_converted_types.empty? %>
#include <<%= typekit.name %>/transports/ros/ROSConvertions.hpp>
<% end %>
<% convert_boxed_types.each do |type, ros_type| %>
#include <<%= ros_message_name(ros_type, true) %>.h>
<% end %>
<% all_messages.each do |msg_name| %>
#include <<%= typekit.name %>_msgs/<%= msg_name %>.h>
<% end %>
<% convert_types.
    find_all { |_, t| t.respond_to?(:deference) && (t.deference <= Typelib::CompoundType || t.deference <= Typelib::OpaqueType) }.
    each do |_, ros_type| %>
#include <<%= ros_message_name(ros_type.deference, true) %>.h>
<% end %>
<% convert_array_types.each do |_, ros_type|
    if ros_type <= Typelib::CompoundType || ros_type <= Typelib::OpaqueType %>
#include <<%= ros_message_name(ros_type, true) %>.h>
<%  end %>
<% end %>

namespace ros_convertions {
    /** Converted types: */
    <% convert_types.each do |type, ros_type| %>
    void toROS( <%= ros_ref_type(ros_type) %> ros, <%= type.arg_type %> value );
    void fromROS( <%= type.ref_type %> value, <%= ros_arg_type(ros_type) %> ros );
    <% end %>
    /** Array types: */
    <% convert_array_types.each do |type, ros_type| %>
    <%   next if type <= Typelib::NumericType  %>
    void toROS( std::vector< <%= ros_cxx_type(ros_type) %> >& ros, <%= type.cxx_name%> const* value, int length );
    void fromROS( <%= type.cxx_name %>* value, std::vector< <%= ros_cxx_type(ros_type) %> > const& ros, int length );
    <% end %>

    <% convert_boxed_types.each do |type, ros_type| %>
    void toROS( <%= ros_ref_type(ros_type, false) %> ros, <%= type.arg_type %> value );
    void fromROS( <%= type.ref_type %> value, <%= ros_arg_type(ros_type, false) %> ros );
    <% end %>
}

#endif


