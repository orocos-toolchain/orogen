/* Generated from orogen/lib/orogen/templates/typekit/corba/TransportPlugin.cpp */

// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include "transports/corba/Registration.hpp"
#include "transports/corba/TransportPlugin.hpp"
#include <rtt/transports/corba/CorbaLib.hpp>
#include <rtt/types/TypekitPlugin.hpp>
using namespace RTT;

bool orogen_typekits::<%= typekit.name %>CorbaTransportPlugin::registerTransport(std::string type_name, RTT::types::TypeInfo* ti)
{
    <% first_type = true;
       typesets.interface_types.each do |type| %>
    <%= 'else ' unless first_type %>if ("<%= type.name %>" == type_name)
    {
        ti->addProtocol(ORO_CORBA_PROTOCOL_ID,
            <%= type.method_name %>_CorbaTransport());
        return true;
    }
    <% first_type = false
    end %>
    return false;
}
std::string orogen_typekits::<%= typekit.name %>CorbaTransportPlugin::getTransportName() const
{ return "CORBA"; }
std::string orogen_typekits::<%= typekit.name %>CorbaTransportPlugin::getTypekitName() const
{ return "/orogen/<%= typekit.name %>"; }
std::string orogen_typekits::<%= typekit.name %>CorbaTransportPlugin::getName() const
{ return "/orogen/<%= typekit.name %>/CORBA"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::<%= typekit.name %>CorbaTransportPlugin);

