/* Generated from orogen/lib/orogen/templates/typekit/mqueue/Registration.hpp */

#ifndef <%= typekit.name.upcase %>_MQUEUE_REGISTRATION_HPP
#define <%= typekit.name.upcase %>_MQUEUE_REGISTRATION_HPP

#include <rtt/types/TypeMarshaller.hpp>

namespace orogen_typekits {
    <% typesets.registered_types.each do |type| %>
    RTT::types::TypeMarshaller*  <%= type.method_name %>_MQueueTransport();
    <% end %>
}

#endif


