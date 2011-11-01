/* Generated from orogen/lib/orogen/templates/typekit/type_info/ArrayInfo.cpp */

#include <<%= typekit.name %>/Types.hpp>
#include <<%= typekit.name %>/type_info/BoostSerialization.hpp>
#include <rtt/internal/carray.hpp>
#include <<%= type.info_type_header %>>

<% base_class =
    if !Orocos::TypekitMarshallers::TypeInfo::Plugin.rtt_scripting?
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

    RTT::types::TypeInfo* <%= type.deference.method_name(true) %>_ArrayTypeInfo()
    { return new <%= type.deference.method_name(true) %>ArrayTypeInfo(); }
}


