#ifndef <%= component.name.upcase %>_TOOLKIT_TYPES_HPP
#define <%= component.name.upcase %>_TOOLKIT_TYPES_HPP

<%= toolkit.opaques.map { |opaque_def| opaque_def.includes }.flatten.map { |p| "#include <#{p}>" }.join("\n") %>

<% for file in loads %>
#line 1 <%= "\"#{file}\"" %>
<%= File.read(file) %>
<% end %>

#endif

