/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <<%= typekit.name %>/Types.hpp>
#include <<%= typekit.name %>/type_info/BoostSerialization.hpp>
#include <<%= type.info_type_header %>>
<% if type.full_name == "/std/string" %>
#include <rtt/typekit/StdStringTypeInfo.hpp>
<% end %>


namespace orogen_typekits {
    struct <%= type.method_name(true) %>TypeInfo :
    <% if type.full_name == "/std/string" %>
        public RTT::types::StdStringTypeInfo
    <% else %>
	public <%= type.info_type %>< <%= type.cxx_name %> >
    <% end %>
    {
        <%= type.method_name(true) %>TypeInfo()
	<% if type.full_name == "/std/string" %>
	    : RTT::types::StdStringTypeInfo("<%= type.full_name %>") {}
        <% else %>
            : <%= type.info_type %>< <%= type.cxx_name %> >("<%= type.full_name %>") {}
	<% end %>
    };

    RTT::types::TypeInfoGenerator* <%= type.method_name(true) %>_TypeInfo()
    { return new <%= type.method_name(true) %>TypeInfo(); }
}

<%= Generation.render_template('typekit', 'TemplateInstanciation.cpp', binding) %>

