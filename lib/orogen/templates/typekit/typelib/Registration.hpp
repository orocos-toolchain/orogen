/* Generated from orogen/lib/orogen/templates/typekit/typelib/Registration.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_TYPELIB_REGISTRATION_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_TYPELIB_REGISTRATION_HPP

#include "transports/typelib/TypelibMarshallerBase.hpp"

namespace orogen_typekits {
    <% typesets.registered_types.each do |type| %>
    /** Creates and returns a TypeInfo object for <%= type.cxx_name %> */
    orogen_transports::TypelibMarshallerBase* <%= type.method_name %>_TypelibMarshaller(Typelib::Registry const& registry);
    <% end %>
}

#endif


