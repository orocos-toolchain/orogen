/* Generated from orogen/lib/orogen/templates/typekit/corba/Registration.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_CORBA_REGISTRATION_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_CORBA_REGISTRATION_HPP

#include <rtt/transports/corba/CorbaTypeTransporter.hpp>

namespace orogen_typekits {
    <% typesets.interface_types.each do |type| %>
    RTT::corba::CorbaTypeTransporter*  <%= type.method_name %>_CorbaTransport();
    <% end %>
}

#endif


