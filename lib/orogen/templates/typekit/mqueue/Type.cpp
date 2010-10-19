/* Generated from orogen/lib/orogen/templates/typekit/mqueue/Type.cpp */

#include "Types.hpp"
#include "boost_serialization.hpp"
#include "transports/mqueue/Registration.hpp"
#include <rtt/transports/mqueue/MQSerializationProtocol.hpp>


namespace orogen_typekits {
    RTT::types::TypeMarshaller*  <%= type.method_name %>_MQueueTransport()
    {
        return new RTT::mqueue::MQSerializationProtocol< <%= type.cxx_name %> >();
    }
}

