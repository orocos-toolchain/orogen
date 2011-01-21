/* Generated from orogen/lib/orogen/templates/typekit/typelib/TransportPlugin.cpp */

#include <typelib/pluginmanager.hh>

#include "Registration.hpp"
#include "TransportPlugin.hpp"
#include "TypelibMarshallerBase.hpp"

#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/Logger.hpp>
using namespace RTT;
#ifdef HAS_ROSLIB
#include <ros/package.h>
#endif

#define TYPEKIT_PACKAGE_NAME_aux0(target) #target
#define TYPEKIT_PACKAGE_NAME_aux(target) "<%= typekit.name %>-typekit-" TYPEKIT_PACKAGE_NAME_aux0(target)
#define TYPEKIT_PACKAGE_NAME TYPEKIT_PACKAGE_NAME_aux(OROCOS_TARGET)

std::string orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::getTlbPath()
{
#ifdef HAS_ROSLIB
    using namespace ros::package;
    try {
        bool all_good = true, found = false;
        std::string ppath = getPath( "<%= typekit.name %>" );
        if ( !ppath.empty() ) {
            return ppath + "/typekit" + "/<%= typekit.name %>.tlb";
        } else
            log(Error) << "Not a ros package: " << "<%= typekit.name %>" << endlog();
    } catch(...) {
        log(Error) << "Not a ros package: " << "<%= typekit.name %>" << endlog();
    }
#endif
    return TYPEKIT_REGISTRY;
}

orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::<%= typekit.name %>TypelibTransportPlugin()
    : m_registry(0)
{
    std::string path = getTlbPath();
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
        log(Error) << "the Typelib transport will not be available for types defined in this typekit" << endlog();
    }
}

orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::~<%= typekit.name %>TypelibTransportPlugin()
{
    delete m_registry;
}

bool orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::registerTransport(std::string type_name, RTT::types::TypeInfo* ti)
{
    if (!m_registry)
        return false;

    <% first_type = true;
       typesets.interface_types.each do |type| %>
    <%= 'else ' unless first_type %>if ("<%= type.name %>" == type_name)
    {
        try {
            ti->addProtocol(orogen_transports::TYPELIB_MARSHALLER_ID,
                <%= type.method_name %>_TypelibMarshaller(*m_registry));
        }
        catch(std::runtime_error const& e)
        {
            log(Error) << "cannot register a typelib transport for " << type_name << endlog();
            log(Error) << "  the following error occured:" << endlog();
            log(Error) << "  " << e.what() << endlog();
            log(Error) << "  the registry can be found at " << TYPEKIT_REGISTRY << endlog();
            return false;
        }
        return true;
    }
    <% first_type = false
    end %>
    return false;
}
std::string orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::getTransportName() const
{ return "Typelib"; }
std::string orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::getTypekitName() const
{ return "/orogen/<%= typekit.name %>"; }
std::string orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::getName() const
{ return "/orogen/<%= typekit.name %>/TYPELIB"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::<%= typekit.name %>TypelibTransportPlugin);

