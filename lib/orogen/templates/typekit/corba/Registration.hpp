#ifndef <%= typekit.name.upcase %>_CORBA_REGISTRATION_HPP
#define <%= typekit.name.upcase %>_CORBA_REGISTRATION_HPP

#include <rtt/transports/corba/CorbaTypeTransporter.hpp>

namespace orogen_typekits {
    <% typesets.registered_types.each do |type| %>
    RTT::corba::CorbaTypeTransporter*  <%= type.method_name %>_CorbaTransport();
    <% end %>
}

#endif


