/* Generated from orogen/lib/orogen/templates/typekit/corba/Convertions.cpp */

#include "<%= typekit.name %>/transports/corba/<%= typekit.name %>TypesC.h"
#include <memory>
// for error-messages in enum-converters "toCORBA()" and "fromCORBA()"
#include <rtt/Logger.hpp>

<% if typekit.has_opaques? %>
#include <<%= typekit.name %>/typekit/OpaqueConvertions.hpp>
<% end %>

<%
# First handle the plain types
needed_convertions = (typesets.converted_types | typesets.array_types).
    inject(Set.new) do |result, type|
        intermediate_type = typekit.intermediate_type_for(type)
        result << type << intermediate_type
        result | type.direct_dependencies.to_set | intermediate_type.direct_dependencies.to_set
    end
needed_convertions.delete_if { |t| t <= Typelib::NumericType }
# Plain types might depend on array types, split the two
needed_array_convertions, needed_convertions = needed_convertions.
    partition { |t| t <= Typelib::ArrayType }
# And add the root array types to the result
needed_array_convertions |= typesets.array_types

# We cannot assume that the headers the user provide do include boost/cstdint.hpp
# So, can't assume that all numeric types are already defined
#
# The only place where it matters, though, is when the int type appears in a
# signature, e.g. when it is the element of a container
needed_numeric_types = needed_convertions.map do |t|
    if t.respond_to?(:deference) && t.deference <= Typelib::NumericType
        t.deference
    end
end.compact
%>

<% all_includes = (needed_numeric_types.to_set | typesets.converted_types | typesets.array_types).flat_map do |type| %>
<%     typekit.include_for_type(type) %>
<% end %>
<%= typekit.cxx_gen_includes(*all_includes) %>

namespace orogen_typekits {
<% needed_convertions.sort_by(&:name).each do |t| %>
    <%= t.to_corba_signature(typekit) %>;
    <%= t.from_corba_signature(typekit) %>;
<% end %>
<% needed_array_convertions.sort_by(&:name).each do |t| %>
    <%= t.to_corba_array_signature(typekit) %>;
    <%= t.from_corba_array_signature(typekit) %>;
<% end %>
}

<% typesets.converted_types.each do |type| %>
<%= type.to_corba_signature(typekit, namespace: "orogen_typekits::") %>
{
<%= result = ""
	type.to_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}

<%= type.from_corba_signature(typekit, namespace: "orogen_typekits::") %>
{
<%= result = ""
	type.from_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>

<% typesets.array_types.each do |type| %>
<%= type.to_corba_array_signature(typekit, namespace: "orogen_typekits::") %>
{
<%= result = ""
	type.to_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<%= type.from_corba_array_signature(typekit, namespace: "orogen_typekits::") %>
{
<%= result = ""
	type.from_corba(typekit, result, " " * 4)
	result 
	%>
    return true;
}
<% end %>

