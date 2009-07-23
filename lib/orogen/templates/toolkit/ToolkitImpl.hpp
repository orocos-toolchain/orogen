#ifndef <%= component.name.upcase %>_TOOLKIT_IMPL_HPP
#define <%= component.name.upcase %>_TOOLKIT_IMPL_HPP

#include "<%= component.name %>ToolkitTypes.hpp"
#include <string>
#include <rtt/PropertyBag.hpp>
#include <rtt/TemplateTypeInfo.hpp>
#include <boost/cstdint.hpp>
#include <iosfwd>

namespace orogen_toolkits {
    <% converted_types.each do |type| %>
    bool toPropertyBag(std::string const& basename, <%= type.arg_type %> value, RTT::PropertyBag& target_bag);
    bool fromPropertyBag(std::string const& basename, <%= type.ref_type %> value, RTT::PropertyBag const& target_bag);
    bool toStream(std::string const& basename, <%= type.arg_type %> value, std::ostream&);
    bool fromStream(std::string const& basename, <%= type.ref_type %> value, std::istream&);
    <% end %>
}

#endif

