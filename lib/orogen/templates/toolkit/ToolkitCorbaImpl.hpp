#ifndef OROCOS_<%= component.name.upcase %>_CORBA_IMPL_HPP
#define OROCOS_<%= component.name.upcase %>_CORBA_IMPL_HPP

#include "<%= component.name %>ToolkitTypes.hpp"
#include <boost/cstdint.hpp>

namespace orogen_toolkits {
    <% converted_types.each do |type| %>
    bool toCORBA( <%= type.corba_ref_type %> corba, <%= type.arg_type %> value );
    bool fromCORBA( <%= type.ref_type %> value, <%= type.corba_arg_type %> corba );
    <% end %>
<% opaques.each do |opdef|
    type = opdef.type
    intermediate_type = component.find_type(opdef.intermediate)
    %>
    bool toCORBA( <%= intermediate_type.corba_ref_type %> corba, <%= type.arg_type %> value );
    bool fromCORBA( <%= type.ref_type %> value, <%= intermediate_type.corba_arg_type %> corba );
<% end %>
}

#endif

