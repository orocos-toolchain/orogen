/* Generated from orogen/lib/orogen/templates/typekit/Types.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_TYPES_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_TYPES_HPP

<% typekit.included_files.to_set.sort.each do |file| %>
#include <<%= file %>>
<% end %>
<% typekit.used_typekits.each do |tk| %>
<% next if tk.virtual? %>
#include <<%= tk.name %>/typekit/Types.hpp>
<% end %>

<% interface_types.each do |type| %>
#ifdef ORO_CHANNEL_ELEMENT_HPP
    extern template class RTT::base::ChannelElement< <%= type.cxx_name %> >;
#endif
#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< <%= type.cxx_name %> >;
    extern template class RTT::internal::AssignableDataSource< <%= type.cxx_name %> >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< <%= type.cxx_name %> >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< <%= type.cxx_name %> >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< <%= type.cxx_name %> >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< <%= type.cxx_name %> >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< <%= type.cxx_name %> >;
#endif
<% end %>

#endif

