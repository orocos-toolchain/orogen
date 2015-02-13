/* Generated from orogen/lib/orogen/templates/typekit/corba/TransportPlugin.hpp */
#pragma once

#include <rtt/types/TransportPlugin.hpp>

namespace orogen_typekits {
    class <%= typekit.name %>CorbaTransportPlugin
        : public RTT::types::TransportPlugin
    {
    public:
        virtual bool registerTransport(std::string type_name, RTT::types::TypeInfo* ti);
        virtual std::string getTransportName() const;
        virtual std::string getTypekitName() const;
        virtual std::string getName() const;
    };

    extern <%= typekit.name %>CorbaTransportPlugin <%= typekit.name %>CorbaTransport;
}


