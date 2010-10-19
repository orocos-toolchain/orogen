/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <<%= typekit.name %>/Types.hpp>
#include <<%= typekit.name %>/boost_serialization.hpp>
#include <<%= type.info_type_header %>>

namespace orogen_typekits {
    struct <%= type.method_name(true) %>TypeInfo :
	public <%= type.info_type %>< <%= type.cxx_name %> >
    {
        <%= type.method_name(true) %>TypeInfo()
            : <%= type.info_type %>< <%= type.cxx_name %> >("<%= type.full_name %>") {}
    };

    RTT::types::TypeInfo* <%= type.method_name(true) %>_TypeInfo()
    { return new <%= type.method_name(true) %>TypeInfo(); }
}

<%= Generation.render_template('typekit', 'TemplateInstanciation.cpp', binding) %>

