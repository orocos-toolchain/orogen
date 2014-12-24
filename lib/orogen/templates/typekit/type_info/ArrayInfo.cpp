/* Generated from orogen/lib/orogen/templates/typekit/type_info/ArrayInfo.cpp */

<%= typekit.cxx_gen_includes(*typekit.include_for_type(type)) %>
<%= typekit.cxx_gen_includes(*typekit.type_info_includes_for_type(type)) %>
#include <rtt/internal/carray.hpp>

<% base_class =
    if !TypekitMarshallers::TypeInfo::Plugin.rtt_scripting?
        "RTT::types::PrimitiveTypeInfo< RTT::internal::carray< #{type.deference.cxx_name} > >"
    else
        "#{type.info_type}< RTT::internal::carray< #{type.deference.cxx_name} > >"
    end
%>

namespace orogen_typekits {
    struct <%= type.deference.method_name(true) %>ArrayTypeInfo :
	public <%= base_class %>
    {
        <%= type.deference.method_name(true) %>ArrayTypeInfo()
            : <%= base_class %>("<%= type.deference.full_name %>[]") {}
    };

    RTT::types::TypeInfoGenerator* <%= type.deference.method_name(true) %>_ArrayTypeInfo()
    { return new <%= type.deference.method_name(true) %>ArrayTypeInfo(); }
}


