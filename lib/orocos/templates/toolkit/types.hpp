#ifndef <%= name.upcase %>_TOOLKIT_TYPES_HPP
#define <%= name.upcase %>_TOOLKIT_TYPES_HPP

namespace <%= name %> {
    <% for file in loads %>
<%= File.read(file) %>
    <% end %>
}

#endif

