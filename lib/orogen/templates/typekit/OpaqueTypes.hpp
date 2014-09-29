#ifndef OROGEN_<%= typekit.name %>_OPAQUE_TYPES_HPP
#define OROGEN_<%= typekit.name %>_OPAQUE_TYPES_HPP

<% opaque_types             = type_sets.opaque_types %>
<% types_containing_opaques = type_sets.types.find_all { |t| t.contains_opaques? && !t.opaque? } %>
<% all_types = opaque_types.map { |op| op.type } + types_containing_opaques %>

<%= all_includes = all_types.inject(Set.new) do |all, type|
       intermediate = typekit.intermediate_type_for(type)
       all |= typekit.include_for_type(type).to_set
       all |= typekit.include_for_type(intermediate).to_set
    end
    typekit.cxx_gen_includes(*all_includes)
%>

#endif
