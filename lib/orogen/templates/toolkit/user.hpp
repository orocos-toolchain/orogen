#ifndef <%= component.name %>_USER_MARSHALLING_HH
#define <%= component.name %>_USER_MARSHALLING_HH

#include <<%= component.name %>ToolkitTypes.hpp>

namespace <%= component.name %>
{
    <% toolkit.marshal_as.each do |from, (into, _)|
        into = component.find_type(into) %>
    void to_intermediate(<%= into.cxx_name %>& intermediate, <%= from.cxx_name %> const& real_type);
    void from_intermediate(<%= from.cxx_name %>& real_type, <%= into.cxx_name %> const& intermediate);
    <% end %>
}

#endif

