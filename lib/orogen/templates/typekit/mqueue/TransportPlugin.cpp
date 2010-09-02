/* Generated from orogen/lib/orogen/templates/typekit/mqueue/TransportPlugin.cpp */

// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include "transports/mqueue/Registration.hpp"
#include "transports/mqueue/TransportPlugin.hpp"
#include <rtt/transports/mqueue/MQLib.hpp>
#include <rtt/types/TypekitPlugin.hpp>
using namespace RTT;

bool orogen_typekits::<%= typekit.name %>MQueueTransportPlugin::registerTransport(std::string type_name, RTT::types::TypeInfo* ti)
{
    <% first_type = true;
       typesets.registered_types.each do |type| %>
    <%= 'else ' unless first_type %>if ("<%= type.name %>" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            <%= type.method_name %>_MQueueTransport());
        return true;
    }
    <% first_type = false
    end %>
    return false;
}
std::string orogen_typekits::<%= typekit.name %>MQueueTransportPlugin::getTransportName() const
{ return "MQueue"; }
std::string orogen_typekits::<%= typekit.name %>MQueueTransportPlugin::getTypekitName() const
{ return "/orogen/<%= typekit.name %>"; }
std::string orogen_typekits::<%= typekit.name %>MQueueTransportPlugin::getName() const
{ return "/orogen/<%= typekit.name %>/MQueue"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::<%= typekit.name %>MQueueTransportPlugin);

