/* Generated from orogen/lib/orogen/templates/typekit/corba/Convertions.hpp */

#ifndef OROCOS_<%= typekit.name.upcase %>_CORBA_CONVERTIONS_HPP
#define OROCOS_<%= typekit.name.upcase %>_CORBA_CONVERTIONS_HPP

#include "Types.hpp"
#include "<%= typekit.name %>/transports/corba/<%= typekit.name %>TypesC.h"
#include <boost/cstdint.hpp>

namespace orogen_typekits {
    /** Converted types: */
    <% typesets.converted_types.each do |type| %>
    bool toCORBA( <%= type.corba_ref_type %> corba, <%= type.arg_type %> value );
    bool fromCORBA( <%= type.ref_type %> value, <%= type.corba_arg_type %> corba );
    <% end %>
    /** Array types: */
    <% typesets.array_types.each do |type| %>
    bool toCORBA( <%= type.corba_ref_type %> corba, <%= type.arg_type %> value, int length );
    bool fromCORBA( <%= type.ref_type %> value, int length, <%= type.corba_arg_type %> corba );
    <% end %>
    /** Opaque types: */
    <% typesets.opaque_types.each do |opdef|
        type = opdef.type
        intermediate_type = typekit.find_type(opdef.intermediate)
        %>
    bool toCORBA( <%= intermediate_type.corba_ref_type %> corba, <%= type.arg_type %> value );
    bool fromCORBA( <%= type.ref_type %> value, <%= intermediate_type.corba_arg_type %> corba );
    <% end %>
}

#endif

