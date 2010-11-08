/* Generated from orogen/lib/orogen/templates/typekit/typelib/TransportPlugin.cpp */

// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include "Registration.hpp"
#include "TransportPlugin.hpp"
#include <rtt/types/TypekitPlugin.hpp>
#include <typelib/pluginmanager.hh>
#include <utilmm/configfile/pkgconfig.hh>
using namespace RTT;

#define TYPEKIT_PACKAGE_NAME_aux0(target) #target
#define TYPEKIT_PACKAGE_NAME_aux(target) "<%= typekit.name %>-typekit-" TYPEKIT_PACKAGE_NAME_aux0(target)
#define TYPEKIT_PACKAGE_NAME TYPEKIT_PACKAGE_NAME_aux(OROCOS_TARGET)

orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::<%= typekit.name %>TypelibTransportPlugin()
    : m_registry(0)
{
    try {
        utilmm::pkgconfig pkg(TYPEKIT_PACKAGE_NAME);
        std::string tlb = pkg.get("type_registry");
        m_registry = Typelib::PluginManager::load("tlb", tlb);
    }
    catch(utilmm::not_found)
    {
        std::cerr << "cannot find the pkg-config specification associated with this typekit:" << std::endl;
        std::cerr << "  " << TYPEKIT_PACKAGE_NAME << std::endl;
        std::cerr << "this is required to use the typelib transport" << std::endl;
        std::cerr << "Remember that you must install the oroGen project with 'make install'" << std::endl;
        std::cerr << "before you can use it, and that PKG_CONFIG_PATH must include" << std::endl;
        std::cerr << "  " << PKG_DIR << std::endl;
    }

}

bool orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::registerTransport(std::string type_name, RTT::types::TypeInfo* ti)
{
    if (!m_registry)
        return false;

    <% first_type = true;
       typesets.registered_types.each do |type| %>
    <%= 'else ' unless first_type %>if ("<%= type.name %>" == type_name)
    {
        ti->addProtocol(orogen_transports::TYPELIB_MARSHALLER_ID,
            <%= type.method_name %>_TypelibMarshaller(*m_registry));
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

