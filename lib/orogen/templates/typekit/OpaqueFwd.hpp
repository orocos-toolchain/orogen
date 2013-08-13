#ifndef OROGEN_<%= typekit.name %>_OPAQUE_FWD_HPP
#define OROGEN_<%= typekit.name %>_OPAQUE_FWD_HPP

<% opaque_types             = type_sets.opaque_types.find_all { |op| !op.generate_templates? } %>
<% types_containing_opaques = type_sets.types.find_all { |t| t.contains_opaques? && !t.opaque? } %>
<% all_types = opaque_types.map { |op| op.type } + types_containing_opaques %>
<% all_types.dup.each do |type|
       all_types << typekit.intermediate_type_for(type)
   end
   templates, classes = all_types.partition do |type|
       type.name =~ /</
   end
%>

<%= template_includes = templates.inject(Set.new) do |all, type|
       all |= typekit.include_for_type(type).to_set
    end
    typekit.cxx_gen_includes(*template_includes)
%>

<% current_namespace = '/' %>
<% classes.sort_by(&:name).each do |type| %>
<%= Orocos::Generation.adapt_namespace(current_namespace, type.namespace) %>
<%     current_namespace = type.namespace %>
class <%= type.basename %>;
<% end %>
<%= Orocos::Generation.adapt_namespace(current_namespace, '/') %>

#endif

