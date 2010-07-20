#ifndef OROCOS_<%= component.name.upcase %>_CORBA_HPP
#define OROCOS_<%= component.name.upcase %>_CORBA_HPP

#include <rtt/TransportPlugin.hpp>

namespace orogen_typekits {
    class <%= component.name %>CorbaTransportPlugin
        : public RTT::TransportPlugin
    {
    public:
        virtual bool registerTransport(std::string type_name, RTT::TypeInfo* ti);
        virtual std::string getTransportName() const;
        virtual std::string getName() const;
    };

    extern <%= component.name %>CorbaTransportPlugin <%= component.name %>CorbaTransport;
}

#endif

