/* Generated from orogen/lib/orogen/templates/typekit/mqueue/TransportPlugin.hpp */
#pragma once

#include <rtt/types/TransportPlugin.hpp>

namespace Typelib {
    class Registry;
}
namespace orogen_typekits {
    class <%= typekit.name %>MQueueTransportPlugin
        : public RTT::types::TransportPlugin
    {
        Typelib::Registry* m_registry;

    public:
        <%= typekit.name %>MQueueTransportPlugin();
        ~<%= typekit.name %>MQueueTransportPlugin();

        virtual bool registerTransport(std::string type_name, RTT::types::TypeInfo* ti);
        virtual std::string getTransportName() const;
        virtual std::string getTypekitName() const;
        virtual std::string getName() const;
    };

    extern <%= typekit.name %>MQueueTransportPlugin <%= typekit.name %>MQueueTransport;
}

