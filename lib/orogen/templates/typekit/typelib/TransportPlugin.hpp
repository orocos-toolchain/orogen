/* Generated from orogen/lib/orogen/templates/typekit/typelib/TransportPlugin.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_TYPELIB_PLUGIN_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_TYPELIB_PLUGIN_HPP

#include <rtt/typelib/TypelibTransportPlugin.hpp>

namespace Typelib
{
    // Forward declaration for the plugin
    class Registry;
}

namespace orogen_typekits {
    class <%= typekit.name %>TypelibTransportPlugin
        : public orogen_transports::TypelibTransportPlugin
    {
    public:
        <%= typekit.name %>TypelibTransportPlugin();
        virtual bool registerTransport(std::string type_name, RTT::types::TypeInfo* ti);
        virtual std::string getTlbPath() const;
    };

    extern <%= typekit.name %>TypelibTransportPlugin <%= typekit.name %>TypelibTransport;
}

#endif

