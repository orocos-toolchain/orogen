/* Generated from orogen/lib/orogen/templates/typekit/corba/Convertions.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_CORBA_CONVERTIONS_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_CORBA_CONVERTIONS_HPP

#include "<%= typekit.name %>/typekit/Types.hpp"
#include "<%= typekit.name %>/transports/corba/<%= typekit.name %>TypesC.h"

<%  type_includes = Set.new
    typesets.converted_types.each do |type| 
        type_includes << typekit.cxx_gen_includes(*typekit.include_for_type(type))
    end %>
<%= type_includes.to_a.join("\n") %>

namespace orogen_typekits {
    /** Converted types: */
    <% typesets.converted_types.each do |type|
        target_type = typekit.intermediate_type_for(type) %>
    bool toCORBA( <%= target_type.corba_ref_type %> corba, <%= type.arg_type %> value );
    bool fromCORBA( <%= type.ref_type %> value, <%= target_type.corba_arg_type %> corba );
    <% end %>
    /** Array types: */
    <% typesets.array_types.each do |type|
        target_type = typekit.intermediate_type_for(type) %>
    bool toCORBA( <%= target_type.corba_ref_type %> corba, <%= type.arg_type %> value, int length );
    bool fromCORBA( <%= type.ref_type %> value, int length, <%= target_type.corba_arg_type %> corba );
    <% end %>
}

#endif

