/* Generated from orogen/lib/orogen/templates/typekit/corba/Registration.hpp */
#pragma once

#include <rtt/transports/corba/CorbaTypeTransporter.hpp>

namespace orogen_typekits {
    <% typesets.interface_types.each do |type| %>
    RTT::corba::CorbaTypeTransporter*  <%= type.method_name %>_CorbaTransport();
    <% end %>
}



