/* Generated from orogen/lib/orogen/templates/typekit/ros/Registration.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_ROS_REGISTRATION_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_ROS_REGISTRATION_HPP

#include <rtt/types/TypeTransporter.hpp>

namespace orogen_typekits {
    <% typesets.interface_types.each do |type|
         next if !ros_exported_type?(type) %>
    RTT::types::TypeTransporter*  <%= type.method_name %>_ROSTransport();
    <% end %>
}

#endif


