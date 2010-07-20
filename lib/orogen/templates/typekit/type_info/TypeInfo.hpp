#ifndef <%= typekit.name.upcase %>_TYPE_INFO_HPP
#define <%= typekit.name.upcase %>_TYPE_INFO_HPP

namespace orogen_typekits {
    <% typesets.registered_types.each do |type| %>
    /** Creates and returns a TypeInfo object for <%= type.cxx_name %> */
    RTT::types::TypeInfo* <%= type.method_name(true) %>_TypeInfo();
    <% end %>
    <% typesets.array_types.each do |type| %>
    /** Creates and returns a TypeInfo object for <%= type.cxx_name %> */
    RTT::types::TypeInfo* <%= type.deference.method_name(true) %>_ArrayTypeInfo();
    <% end %>
}

#endif

