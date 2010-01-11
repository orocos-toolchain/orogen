#ifndef <%= component.name.upcase %>_TOOLKIT_TYPES_HPP
#define <%= component.name.upcase %>_TOOLKIT_TYPES_HPP

// This is a hack. We include it unconditionally as it may be required by some
// toolkits *and* it is a standard header. Ideally, we would actually check if
// some of the types need std::vector.
#include <vector>
#include <boost/cstdint.hpp>

<%= toolkit.opaques.map { |opaque_def| opaque_def.includes }.flatten.map { |p| "#include <#{p}>" }.join("\n") %>

<% toolkit.external_loads.each do |file| %>
#include <<%= file %>>
<% end %>
<% toolkit.local_headers(false).each do |file|
    file = file.gsub(/^#{component.name}\//, '') %>
#include "<%= component.name %>/<%= file %>"
<% end %>
<% toolkit.used_toolkits.each do |tk| %>
#include <orocos/toolkit/<%= tk.name %>ToolkitTypes.hpp>
<% end %>

<% registered_types.each do |type| %>
#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::DataSource< <%= type.cxx_name %> >;
    extern template class RTT::ValueDataSource< <%= type.cxx_name %> >;
    extern template class RTT::ConstantDataSource< <%= type.cxx_name %> >;
    extern template class RTT::AssignableDataSource< <%= type.cxx_name %> >;
    extern template class RTT::ReferenceDataSource< <%= type.cxx_name %> >;
    extern template class RTT::detail::DataSourceAdaptor< <%= type.cxx_name %> const, <%= type.cxx_name %> >;
    extern template class RTT::detail::DataSourceAdaptor< <%= type.cxx_name %> const &, <%= type.cxx_name %> >;
    extern template class RTT::detail::DataSourceAdaptor< <%= type.cxx_name %>&, <%= type.cxx_name %> >;
#endif
#ifdef ORO_EXECUTION_PORTS_HPP
    extern template class RTT::OutputPort< <%= type.cxx_name %> >;
    extern template class RTT::InputPort< <%= type.cxx_name %> >;
#endif
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

