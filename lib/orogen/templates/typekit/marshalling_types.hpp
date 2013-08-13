/* Generated from orogen/lib/orogen/templates/typekit/marshalling_types.hpp */

#ifndef _OROGEN_M_TYPES_<%= type.method_name(false).upcase %>_HPP
#define _OROGEN_M_TYPES_<%= type.method_name(false).upcase %>_HPP

<% all_includes = needed_type_definitions.inject(Set.new) do |all, needed_type| %>
<%     all |= typekit.include_for_type(needed_type).to_set %>
<% end %>
<%= typekit.cxx_gen_includes(*all_includes) %>

<% target_typename = typekit.intermediate_type_name_for(type)
   target_namespace = Typelib.namespace(target_typename)
   target_basename  = Typelib.basename(target_typename)
   target_basename.gsub!('/', '::')
%>
<%= Generation.adapt_namespace('/', target_namespace) %>
<%= type.to_m_type(target_basename, typekit) %>
<%= Generation.adapt_namespace(target_namespace, '/') %>

#endif
