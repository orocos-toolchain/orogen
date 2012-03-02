/* Generated from orogen/lib/orogen/templates/typekit/mqueue/TransportPlugin.cpp */

// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include "transports/mqueue/Registration.hpp"
#include "transports/mqueue/TransportPlugin.hpp"
#include <rtt/transports/mqueue/MQLib.hpp>
#include <rtt/types/TypekitPlugin.hpp>
using namespace RTT;

#include "transports/typelib/TransportPlugin.hpp"
#include <typelib/pluginmanager.hh>
#include <typelib/registry.hh>
#include <rtt/Logger.hpp>

orogen_typekits::<%= typekit.name %>MQueueTransportPlugin::<%= typekit.name %>MQueueTransportPlugin()
    : m_registry(0)
{
    std::string path = <%= typekit.name %>TypelibTransportPlugin::getTypelibRegistryPath();
    try
    {
        m_registry = Typelib::PluginManager::load("tlb", path);
    }
    catch(std::exception const& e) {
        log(Error) << "cannot load the typekit's Typelib registry from" << endlog();
        log(Error) << "  " << path << endlog();
#ifndef HAS_ROSLIB
        log(Error) << "remember to do 'make install' before you use the oroGen-generated libraries ?" << endlog();
#endif
        log(Error) << endlog();
        log(Error) << "the MQueue transport will not be available for types defined in this typekit" << endlog();
    }
}

orogen_typekits::<%= typekit.name %>MQueueTransportPlugin::~<%= typekit.name %>MQueueTransportPlugin()
{
    delete m_registry;
}

bool orogen_typekits::<%= typekit.name %>MQueueTransportPlugin::registerTransport(std::string type_name, RTT::types::TypeInfo* ti)
{
    if (!m_registry)
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
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            <%= type.method_name %>_MQueueTransport(*m_registry));
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

