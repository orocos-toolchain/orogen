#ifndef <%= component.name %>_USER_MARSHALLING_HH
#define <%= component.name %>_USER_MARSHALLING_HH

#include <<%= component.name %>ToolkitTypes.hpp>

namespace <%= component.name %>
{
    <% toolkit.opaques.each do |opaque_def|
        from = opaque_def.type
        into = component.find_type(opaque_def.intermediate) %>
    void to_intermediate(<%= into.cxx_name %>& intermediate, <%= from.cxx_name %> const& real_type);
    void from_intermediate(<%= from.cxx_name %>& real_type, <%= into.cxx_name %> const& intermediate);
    <% end %>
}

#endif

