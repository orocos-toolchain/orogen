/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <<%= typekit.name %>/Types.hpp>
#include <<%= typekit.name %>/type_info/BoostSerialization.hpp>
#include <<%= type.info_type_header %>>
<% if type.full_name == "/std/string" %>
#include <rtt/typekit/StdStringTypeInfo.hpp>
<% end %>

<% base_class =
    if !Orocos::TypekitMarshallers::TypeInfo::Plugin.rtt_scripting?
        ["RTT::types::PrimitiveTypeInfo< #{type.cxx_name} >", "RTT::types::TemplateConnFactory< #{type.cxx_name} >"]
    elsif type.full_name == "/std/string"
        ["RTT::types::StdStringTypeInfo"]
    else
	["#{type.info_type}< #{type.cxx_name} >"]
    end
%>

namespace orogen_typekits {
    struct <%= type.method_name(true) %>TypeInfo :
        public <%= base_class.join(", public ") %>
    {
        <%= type.method_name(true) %>TypeInfo()
            : <%= base_class.first %>("<%= type.full_name %>") {}

<%  if !Orocos::TypekitMarshallers::TypeInfo::Plugin.rtt_scripting? %>
        bool installTypeInfoObject(RTT::types::TypeInfo* ti) {
            // Allow base to install first
            RTT::types::PrimitiveTypeInfo< <%= type.cxx_name %> >::installTypeInfoObject(ti);
            // Install the factories for primitive types
            ti->setPortFactory( boost::dynamic_pointer_cast< RTT::types::TemplateConnFactory< <%= type.cxx_name %> > >( this->getSharedPtr() ) );

            return false;
        }
<% end %>

    };

    RTT::types::TypeInfoGenerator* <%= type.method_name(true) %>_TypeInfo()
    { return new <%= type.method_name(true) %>TypeInfo(); }

}

<%= Generation.render_template('typekit', 'TemplateInstanciation.cpp', binding) %>

