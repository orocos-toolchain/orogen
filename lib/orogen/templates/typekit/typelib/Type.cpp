/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include "Types.hpp"
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::<%= type.method_name %>_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< <%= type.cxx_name %> >("<%= type.name %>", registry);
}



