#ifndef <%= component.name.upcase %>_TOOLKIT_IMPL_HPP
#define <%= component.name.upcase %>_TOOLKIT_IMPL_HPP

#include "<%= component.name %>TypekitTypes.hpp"
#include <string>
#include <rtt/PropertyBag.hpp>
#include <rtt/TemplateTypeInfo.hpp>
#include <boost/cstdint.hpp>
#include <iosfwd>

namespace orogen_typekits {
    <% (converted_types + opaque_types).each do |type| %>
    bool toPropertyBag(std::string const& basename, <%= type.arg_type %> value, RTT::PropertyBag& target_bag);
    bool fromPropertyBag(std::string const& basename, <%= type.ref_type %> value, RTT::PropertyBag const& target_bag);
    bool toStream(std::string const& basename, <%= type.arg_type %> value, std::ostream&);
    bool fromStream(std::string const& basename, <%= type.ref_type %> value, std::istream&);
    <% end %>
    <% array_types.each do |type| %>
    bool toPropertyBag(std::string const& basename, <%= type.arg_type %> value, int length, RTT::PropertyBag& target_bag);
    bool fromPropertyBag(std::string const& basename, <%= type.ref_type %> value, int length, RTT::PropertyBag const& target_bag);
    bool toStream(std::string const& basename, <%= type.arg_type %> value, int length, std::ostream&);
    bool fromStream(std::string const& basename, <%= type.ref_type %> value, int length, std::istream&);
    <% end %>
}

#endif

