#include <rtt/Ports.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Properties.hpp>
#include <rtt/DataSource.hpp>

#include "<%= name %>TypekitTypes.hpp"

template class RTT::OutputPort< <%= type.cxx_name %> >;
template class RTT::InputPort< <%= type.cxx_name %> >;
template class RTT::Property< <%= type.cxx_name %> >;
template class RTT::Attribute< <%= type.cxx_name %> >;
template class RTT::DataSource< <%= type.cxx_name %> >;
template class RTT::ValueDataSource< <%= type.cxx_name %> >;
template class RTT::ConstantDataSource< <%= type.cxx_name %> >;
template class RTT::AssignableDataSource< <%= type.cxx_name %> >;
template class RTT::ReferenceDataSource< <%= type.cxx_name %> >;
template class RTT::detail::DataSourceAdaptor< <%= type.cxx_name %> const, <%= type.cxx_name %> >;
template class RTT::detail::DataSourceAdaptor< <%= type.cxx_name %> const &, <%= type.cxx_name %> >;
template class RTT::detail::DataSourceAdaptor< <%= type.cxx_name %>&, <%= type.cxx_name %> >;

