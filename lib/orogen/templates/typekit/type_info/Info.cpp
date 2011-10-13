/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <<%= typekit.name %>/Types.hpp>
#include <<%= typekit.name %>/type_info/BoostSerialization.hpp>
#include <<%= type.info_type_header %>>
<% if type.full_name == "/std/string" %>
#include <rtt/typekit/StdStringTypeInfo.hpp>
<% end %>

<% base_class =
    if !Orocos::TypekitMarshallers::TypeInfo::Plugin.rtt_scripting?
        "RTT::types::TemplateTypeInfoBase< #{type.cxx_name} >"
    elsif type.full_name == "/std/string"
        "RTT::types::StdStringTypeInfo"
    else
	"#{type.info_type}< #{type.cxx_name} >"
    end
%>

namespace orogen_typekits {
    struct <%= type.method_name(true) %>TypeInfo :
        public <%= base_class %>
    {
        <%= type.method_name(true) %>TypeInfo()
            : <%= base_class %>("<%= type.full_name %>") {}
    };

    RTT::types::TypeInfo* <%= type.method_name(true) %>_TypeInfo()
    { return new <%= type.method_name(true) %>TypeInfo(); }
}

<%= Generation.render_template('typekit', 'TemplateInstanciation.cpp', binding) %>

