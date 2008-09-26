#ifndef <%= component.name.upcase %>_TOOLKIT_TYPES_HPP
#define <%= component.name.upcase %>_TOOLKIT_TYPES_HPP

<% for file in loads %>
<%= File.read(file) %>
<% end %>

#endif

