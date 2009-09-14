#ifndef <%= component.name.upcase %>_TOOLKIT_TYPES_HPP
#define <%= component.name.upcase %>_TOOLKIT_TYPES_HPP

// This is a hack. We include it unconditionally as it may be required by some
// toolkits *and* it is a standard header. Ideally, we would actually check if
// some of the types need std::vector.
#include <vector>

<%= toolkit.opaques.map { |opaque_def| opaque_def.includes }.flatten.map { |p| "#include <#{p}>" }.join("\n") %>

<%
    local_headers = toolkit.local_headers
    for file in loads %>
<% if local_headers.include?(file) %>
#include "<%= component.name %>/<%= File.basename(file) %>"
<% else %>
#include <<%= file %>>
<% end %>

<% end %>

<% generated_types.each do |type|
    next if toolkit.m_type?(type) %>
#ifdef ORO_EXECUTION_PORTS_HPP
    extern template class RTT::OutputPort< <%= type.cxx_name %> >;
    extern template class RTT::InputPort< <%= type.cxx_name %> >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< <%= type.cxx_name %> >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< <%= type.cxx_name %> >;
#endif
<% end %>
                
<%= 
    catch(:nothing_to_define) do
        generate_all_marshalling_types = true
        Generation.render_template 'toolkit/marshalling_types.hpp', binding
    end %>

#endif

