#ifndef <%= component.name %>_USER_MARSHALLING_HH
#define <%= component.name %>_USER_MARSHALLING_HH

#include <<%= component.name %>ToolkitTypes.hpp>

<%= toolkit.marshal_as.map { |_, (_, includes)| includes }.flatten.map { |p| "#include <#{p}>" }.join("\n") %>

class <%= component.name %>UserMarshalling
{
public:

<% toolkit.marshal_as.each do |from, (into, _)|
    into = component.find_type(into) %>
    void dump_<%= from.method_name %>(<%= from.cxx_name %> const& real_type, <%= into.cxx_name %>& intermediate);
    void load_<%= from.method_name %>(<%= from.cxx_name %>& real_type, <%= into.cxx_name %> const& intermediate);
<% end %>
};

#endif

