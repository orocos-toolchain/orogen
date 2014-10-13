/* Generated from orogen/lib/orogen/templates/typekit/BoostSerialization.hpp */

#ifndef __OROGEN_GENERATED_<%= typekit.name.upcase %>_BOOST_SERIALIZATION_HPP
#define __OROGEN_GENERATED_<%= typekit.name.upcase %>_BOOST_SERIALIZATION_HPP

#include <<%= typekit.name %>/typekit/Types.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/utility.hpp>

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

