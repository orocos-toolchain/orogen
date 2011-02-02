/* Generated from orogen/lib/orogen/templates/typekit/typelib/OpaqueType.cpp */

#include "OpaqueConvertions.hpp"
#include "Types.hpp"
#include <rtt/typelib/OpaqueTypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::<%= type.method_name %>_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::OpaqueTypelibMarshaller< <%= intermediate.cxx_name %>, <%= type.cxx_name %>, <%= needs_copy %> >("<%= intermediate.cxx_name %>", "<%= type.name %>", registry);
}

