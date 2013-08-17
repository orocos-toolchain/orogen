/* Generated from orogen/lib/orogen/templates/typekit/typelib/OpaqueType.cpp */

<%= typekit.cxx_gen_includes(*typekit.include_for_type(type)) %>
<%= typekit.cxx_gen_includes(*typekit.include_for_type(typekit.intermediate_type_for(type))) %>
#include <<%= typekit.name %>/typekit/OpaqueConvertions.hpp>
#include <rtt/typelib/OpaqueTypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::<%= type.method_name %>_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::OpaqueTypelibMarshaller< <%= intermediate.cxx_name %>, <%= type.cxx_name %>, <%= needs_copy %> >("<%= intermediate.name %>", "<%= type.name %>", registry);
}

