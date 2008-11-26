#ifndef <%= component.name.upcase %>_TOOLKIT_TYPES_HPP
#define <%= component.name.upcase %>_TOOLKIT_TYPES_HPP

<% for file in loads %>
<% if File.exists?(file) %>
<%= File.read(file) %>
<% else %>
#line 1 <%= "\"#{file}\"" %>
#include <<%= file %>>
<% end %>

<% end %>

#endif

