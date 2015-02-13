/* Generated from orogen/lib/orogen/templates/typekit/mqueue/Registration.hpp */
#pragma once

#include <rtt/types/TypeMarshaller.hpp>

namespace Typelib {
    class Registry;
}
namespace orogen_typekits {
    <% typesets.interface_types.each do |type| %>
    RTT::types::TypeMarshaller*  <%= type.method_name %>_MQueueTransport(Typelib::Registry const& registry);
    <% end %>
}

