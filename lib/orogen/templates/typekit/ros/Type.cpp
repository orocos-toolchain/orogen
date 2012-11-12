/* Generated from orogen/lib/orogen/templates/typekit/ros/Type.cpp */

#include "Types.hpp"
#include "transports/ros/Registration.hpp"
#include "transports/ros/Convertions.hpp"
#include <rtt/transports/ros/ros_msg_transporter.hpp>

<% if type.respond_to?(:deference) %>
#include <<%= ros_message_name(type.deference, true) %>.h>
<% else %>
#include <<%= ros_message_name(type, true) %>.h>
<% end %>

namespace orogen_typekits {
    RTT::types::TypeTransporter*  <%= type.method_name %>_ROSTransport()
    {
        return new ros_integration::RosMsgTransporter< <%= type.cxx_name %>, <%= ros_cxx_type(type, false) %> >;
    }
}

