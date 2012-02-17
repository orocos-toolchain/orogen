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
    if(ti->hasProtocol(ORO_CORBA_PROTOCOL_ID))
	return false;

    <% first_type = true;
       typesets.interface_types.each do |type|
           names = [type.name]
           if aliases = typesets.aliases[type]
               names.concat(aliases)
           end
           if_cond = names.map { |n| "\"#{n}\" == type_name" }.join(" || ")
        %>
    <%= 'else ' unless first_type %>if (<%= if_cond %>)
    {
        return ti->addProtocol(ORO_CORBA_PROTOCOL_ID,
            <%= type.method_name %>_CorbaTransport());
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

