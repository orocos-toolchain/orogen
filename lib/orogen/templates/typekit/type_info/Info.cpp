#include <<%= typekit.name %>/Types.hpp>
#include <<%= type.info_type_header %>>

<% if type.respond_to?(:to_boost_serialization) %>
namespace boost
{
    namespace serialization
    {
        template<typename Archive>
        void serialize(Archive& a, <%= type.cxx_name %>& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            <%= type.to_boost_serialization %>
        }
    }
}
<% end %>

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

