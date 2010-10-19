/*
 * Generated from orogen/lib/templates/typekit/boost_serialization.hpp
 *
 */

#ifndef OROGEN_<%= typekit.name.upcase %>_BOOST_SERIALIZATION_HPP_
#define OROGEN_<%= typekit.name.upcase %>_BOOST_SERIALIZATION_HPP_

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>

#include "Types.hpp"

<% registered_types.each do |type| %>
<% if type.respond_to?(:to_boost_serialization) %>
namespace boost
{
    namespace serialization
    {
        /**
         * boost::serialization function for <%= type.cxx_name %>
         */
        template<typename Archive>
        void serialize(Archive& a, <%= type.cxx_name %>& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            <%= type.to_boost_serialization %>
        }
    }
}
<% end %>
<% end %>

#endif /* OROGEN_BOOST_SERIALIZATION_HPP_ */
