/* Generated from orogen/lib/orogen/templates/typekit/typelib/Registration.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_TYPELIB_REGISTRATION_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_TYPELIB_REGISTRATION_HPP

#include <rtt/types/TypeInfoGenerator.hpp>

namespace Typelib {
    class Registry;
}

namespace orogen_transports {
    class TypelibMarshallerBase;
}

namespace orogen_typekits {
    <% typesets.interface_types.each do |type| %>
    /** Creates and returns a TypeInfo object for <%= type.cxx_name %> */
    orogen_transports::TypelibMarshallerBase* <%= type.method_name %>_TypelibMarshaller(Typelib::Registry const& registry);
    <% end %>
}

#endif


