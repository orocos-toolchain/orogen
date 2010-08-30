/* Generated from orogen/lib/orogen/templates/typekit/typelib/TransportPlugin.hpp */

#ifndef OROCOS_<%= typekit.name.upcase %>_TYPELIB_PLUGIN_HPP
#define OROCOS_<%= typekit.name.upcase %>_TYPELIB_PLUGIN_HPP

#include <rtt/types/TransportPlugin.hpp>

namespace Typelib
{
    // Forward declaration for the plugin
    class Registry;
}

namespace orogen_typekits {
    class <%= typekit.name %>TypelibTransportPlugin
        : public RTT::types::TransportPlugin
    {
        Typelib::Registry* m_registry;

    public:
        <%= typekit.name %>TypelibTransportPlugin();
        virtual bool registerTransport(std::string type_name, RTT::types::TypeInfo* ti);
        virtual std::string getTransportName() const;
        virtual std::string getTypekitName() const;
        virtual std::string getName() const;
    };

    extern <%= typekit.name %>TypelibTransportPlugin <%= typekit.name %>TypelibTransport;
}

#endif

