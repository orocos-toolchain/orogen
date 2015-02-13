/* Generated from orogen/lib/orogen/templates/typekit/type_info/TypeInfo.hpp */
#pragma once

namespace orogen_typekits {
    <% typesets.registered_types.each do |type| %>
        <% if type < Typelib::ArrayType %>
    /** Creates and returns a TypeInfo object for <%= type.cxx_name %> */
    RTT::types::TypeInfoGenerator* <%= type.deference.method_name(true) %>_ArrayTypeInfo();
        <% else %>
    /** Creates and returns a TypeInfo object for <%= type.cxx_name %> */
    RTT::types::TypeInfoGenerator* <%= type.method_name(true) %>_TypeInfo();
        <% end %>
    <% end %>
}

