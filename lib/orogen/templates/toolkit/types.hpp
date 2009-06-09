#ifndef <%= component.name.upcase %>_TOOLKIT_TYPES_HPP
#define <%= component.name.upcase %>_TOOLKIT_TYPES_HPP

<%= toolkit.opaques.map { |opaque_def| opaque_def.includes }.flatten.map { |p| "#include <#{p}>" }.join("\n") %>

<% for file in loads %>
<% if File.exists?(file) %>
<%= File.read(file) %>
<% else %>
#line 1 <%= "\"#{file}\"" %>
#include <<%= file %>>
<% end %>

<% end %>

<% generated_types.each do |type|
    next if toolkit.m_type?(type) %>
#ifdef ORO_EXECUTION_PORTS_HPP
    extern template class RTT::OutputPort<<%= type.cxx_name %>>;
    extern template class RTT::InputPort<<%= type.cxx_name %>>;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property<<%= type.cxx_name %>>;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute<<%= type.cxx_name %>>;
#endif
<% end %>
                
<%= 
    catch(:nothing_to_define) do
        generate_all_marshalling_types = true
        Generation.render_template 'toolkit/marshalling_types.hpp', binding
    end %>

#endif

