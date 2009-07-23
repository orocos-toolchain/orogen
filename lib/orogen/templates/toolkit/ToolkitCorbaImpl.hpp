#ifndef OROCOS_<%= component.name.upcase %>_CORBA_IMPL_HPP
#define OROCOS_<%= component.name.upcase %>_CORBA_IMPL_HPP

#include "<%= component.name %>ToolkitTypes.hpp"
#include "<%= component.name %>ToolkitC.h"
#include <rtt/corba/CorbaTemplateProtocol.hpp>
#include <boost/cstdint.hpp>

namespace orogen_toolkits {
    <% converted_types.each do |type| %>
    bool toCORBA( <%= type.corba_ref_type %> corba, <%= type.arg_type %> value );
    bool fromCORBA( <%= type.corba_arg_type %> corba, <%= type.ref_type %> value );
    <% end %>
}

#endif

