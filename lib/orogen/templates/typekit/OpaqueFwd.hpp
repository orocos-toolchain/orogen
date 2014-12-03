#ifndef OROGEN_<%= typekit.name %>_OPAQUE_FWD_HPP
#define OROGEN_<%= typekit.name %>_OPAQUE_FWD_HPP

<% opaque_types             = type_sets.opaque_types %>
<% types_containing_opaques = type_sets.types.find_all { |t| t.contains_opaques? && !t.opaque? } %>
<% all_types = opaque_types.map { |op| op.type } + types_containing_opaques
   all_types.dup.each do |type|
       all_types << typekit.intermediate_type_for(type)
   end
   all_types.map! do |type|
       if type <= Typelib::ArrayType
           while type.respond_to?(:deference)
               type = type.deference
           end
       end
       type
   end
   need_full_definition, forward_declared = all_types.partition do |type|
       type.name =~ /</ || (type.opaque? && type.metadata.get('opaque_is_typedef').include?('1'))
   end
%>

<%= includes = need_full_definition.inject(Set.new) do |all, type|
       all |= typekit.include_for_type(type).to_set
    end
    typekit.cxx_gen_includes(*includes)
%>

<% current_namespace = '/' %>
<% forward_declared.sort_by(&:name).each do |type| %>
<%= Orocos::Generation.adapt_namespace(current_namespace, type.namespace) %>
<%     current_namespace = type.namespace %>
class <%= type.basename %>;
<% end %>
<%= Orocos::Generation.adapt_namespace(current_namespace, '/') %>

#endif

