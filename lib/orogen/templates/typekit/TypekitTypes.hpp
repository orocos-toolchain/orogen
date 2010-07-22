#ifndef <%= typekit.name.upcase %>_TOOLKIT_TYPES_HPP
#define <%= typekit.name.upcase %>_TOOLKIT_TYPES_HPP

<%= typekit.opaques.map { |opaque_def| opaque_def.includes }.flatten.map { |p| "#include <#{p}>" }.join("\n") %>

<% typekit.external_loads.each do |file| %>
#include <<%= file %>>
<% end %>
<% typekit.local_headers(false).each do |path, dest_path| %>
#include "<%= File.join(typekit.name, dest_path) %>"
<% end %>
<% typekit.used_typekits.each do |tk| %>
#include <typekit/<%= tk.name %>TypekitTypes.hpp>
<% end %>

// This is a hack. We include it unconditionally as it may be required by some
// typekits *and* it is a standard header. Ideally, we would actually check if
// some of the types need std::vector.
#include <vector>
#include <boost/cstdint.hpp>

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
        Generation.render_template 'typekit/marshalling_types.hpp', binding
    end %>

#endif

