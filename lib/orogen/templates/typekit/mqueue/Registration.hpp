/* Generated from orogen/lib/orogen/templates/typekit/mqueue/Registration.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_MQUEUE_REGISTRATION_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_MQUEUE_REGISTRATION_HPP

#include <rtt/types/TypeMarshaller.hpp>

namespace orogen_typekits {
    <% mqueue_registered_types.each do |type| %>
    RTT::types::TypeMarshaller*  <%= type.method_name %>_MQueueTransport();
    <% end %>
}

#endif


