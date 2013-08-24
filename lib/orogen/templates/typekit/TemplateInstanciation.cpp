/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< <%= type.cxx_name %> >;
template class RTT::internal::DataSource< <%= type.cxx_name %> >;
template class RTT::internal::AssignableDataSource< <%= type.cxx_name %> >;
template class RTT::internal::ValueDataSource< <%= type.cxx_name %> >;
template class RTT::OutputPort< <%= type.cxx_name %> >;
template class RTT::InputPort< <%= type.cxx_name %> >;
template class RTT::Property< <%= type.cxx_name %> >;
template class RTT::Attribute< <%= type.cxx_name %> >;


