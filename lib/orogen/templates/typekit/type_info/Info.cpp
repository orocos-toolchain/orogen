/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <<%= typekit.name %>/Types.hpp>
#include <<%= typekit.name %>/type_info/BoostSerialization.hpp>
#include <<%= type.info_type_header %>>
<% if type.full_name == "/std/string" %>
#include <rtt/typekit/StdStringTypeInfo.hpp>
<% end %>

<% base_class =
    if !Orocos::TypekitMarshallers::TypeInfo::Plugin.rtt_scripting?
        "RTT::types::PrimitiveTypeInfo< #{type.cxx_name} >, RTT::types::TemplateConnFactory< #{type.cxx_name} >"
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

    RTT::types::TypeInfoGenerator* <%= type.method_name(true) %>_TypeInfo()
    { return new <%= type.method_name(true) %>TypeInfo(); }

<%  if !Orocos::TypekitMarshallers::TypeInfo::Plugin.rtt_scripting? %>
    bool installTypeInfoObject(TypeInfo* ti) {
        // aquire a shared reference to the this object
        boost::shared_ptr< TemplateTypeInfo< <%= type.cxx_name %> > mthis =
            boost::dynamic_pointer_cast<TemplateTypeInfo<<%= type.cxx_name %>> >( this->getSharedPtr() );
        assert(mthis);
        // Allow base to install first
        PrimitiveTypeInfo<<%= type.cxx_name %>>::installTypeInfoObject(ti);
        // Install the factories for primitive types
        ti->setPortFactory( mthis );

        return false;
    }
<% end %>

}

<%= Generation.render_template('typekit', 'TemplateInstanciation.cpp', binding) %>

