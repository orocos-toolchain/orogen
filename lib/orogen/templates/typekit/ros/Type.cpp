/* Generated from orogen/lib/orogen/templates/typekit/ros/Type.cpp */

#include <<%= typekit.name %>/typekit/Types.hpp>
#include "transports/ros/Registration.hpp"
#include "transports/ros/Convertions.hpp"
#include <rtt/transports/ros/ros_msg_transporter.hpp>

#include <<%= ros_message_name(type, true) %>.h>

namespace orogen_typekits {
    RTT::types::TypeTransporter*  <%= type.method_name %>_ROSTransport()
    {
        return new RTT::ros::RosMsgTransporter< <%= type.cxx_name %>, <%= ros_cxx_type(type, false) %> >;
    }
}

