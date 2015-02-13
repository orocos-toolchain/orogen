/* Generated from orogen/lib/orogen/templates/typekit/ros/Registration.hpp */
#pragma once

#include <rtt/types/TypeTransporter.hpp>

namespace orogen_typekits {
    <% typesets.interface_types.each do |type|
         next if !ros_exported_type?(type) %>
    RTT::types::TypeTransporter*  <%= type.method_name %>_ROSTransport();
    <% end %>
}

