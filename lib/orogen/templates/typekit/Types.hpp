/* Generated from orogen/lib/orogen/templates/typekit/Types.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_TYPES_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_TYPES_HPP

<%= typekit.opaques.map { |opaque_def| opaque_def.includes }.
    flatten.map { |p| "#include <#{p}>" }.join("\n") %>

<% typekit.external_loads.each do |file| %>
#include <<%= file %>>
<% end %>
<% typekit.local_headers(false).each do |path, dest_path| %>
#include "<%= typekit.name %>/types/<%= typekit.name %>/<%= dest_path %>"
<% end %>
<% typekit.used_typekits.each do |tk| %>
<% next if tk.virtual? %>
#include <<%= tk.name %>/Types.hpp>
<% end %>

// This is a hack. We include it unconditionally as it may be required by some
// typekits *and* it is a standard header. Ideally, we would actually check if
// some of the types need std::vector.
#include <vector>
#include <boost/cstdint.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/utility.hpp>

<%= typekit.m_types_code %>

<% interface_types.each do |type| %>
#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< <%= type.cxx_name %> >;
    extern template class RTT::internal::AssignableDataSource< <%= type.cxx_name %> >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< <%= type.cxx_name %> >;
    extern template class RTT::internal::ConstantDataSource< <%= type.cxx_name %> >;
    extern template class RTT::internal::ReferenceDataSource< <%= type.cxx_name %> >;
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

<% boost_serialize_types = converted_types.
    find_all do |t|
        t.boost_serialization_compatible? &&
            t.respond_to?(:to_boost_serialization)
    end %>
<% if !boost_serialize_types.empty? %>
namespace boost
{
    namespace serialization
    {
<%    boost_serialize_types.each do |type| %>
        template<typename Archive>
        void serialize(Archive& a, <%= type.cxx_name %>& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            <%= type.to_boost_serialization %>
        }
<%    end %>
    }
}
<% end %>

#endif

