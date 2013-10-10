/* Generated from orogen/lib/orogen/templates/typekit/mqueue/Type.cpp */

<%= typekit.cxx_gen_includes(*typekit.include_for_type(type)) %>
#include "transports/mqueue/Registration.hpp"

<% use_plain_mempcy = type.compatible_with_memcpy? %>
<% if use_plain_mempcy %>
#include <rtt/transports/mqueue/MQTemplateProtocol.hpp>
<% else %>
#include "transports/typelib/Registration.hpp"
#include <rtt/typelib/MQTypelibMarshaller.hpp>
<% end %>

namespace Typelib
{
    class Registry;
}

namespace orogen_typekits {
    RTT::types::TypeMarshaller*  <%= type.method_name %>_MQueueTransport(Typelib::Registry const& registry)
    {
        <% if use_plain_mempcy %>
        return new RTT::mqueue::MQTemplateProtocol< <%= type.cxx_name %> >();
        <% else %>
        orogen_transports::TypelibMarshallerBase* marshaller =
            <%= type.method_name %>_TypelibMarshaller(registry);

        return new orogen_transports::MQTypelibMarshaller< <%= type.cxx_name %> >(marshaller);
        <% end %>
    }
}

