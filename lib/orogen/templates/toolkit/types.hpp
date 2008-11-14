#ifndef <%= component.name.upcase %>_TOOLKIT_TYPES_HPP
#define <%= component.name.upcase %>_TOOLKIT_TYPES_HPP

<% for file in loads %>
#line 1 <%= "\"#{file}\"" %>
<%= File.read(file) %>
<% end %>

#endif

