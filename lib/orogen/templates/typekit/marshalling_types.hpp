/* Generated from orogen/lib/orogen/templates/typekit/marshalling_types.hpp */

<% namespace = '/'
   did_something = false %>

<% typekit.loads.each do |path| %>
#include "<%= path %>"
<% end %>
<% typekit.local_headers(false).each do |path, dest_path| %>
#include "<%= File.join(typekit.name, dest_path) %>"
<% end %>
<% typekit.used_typekits.each do |tk| %>
<% next if tk.virtual? %>
#include <<%= tk.name %>/Types.hpp>
<% end %>

<% needed_types.each do |type|
    current_def = begin
                      registry.get("#{type.full_name}_m")
                  rescue Typelib::NotFound
                  end

    if !generate_all_marshalling_types && current_def
        # Check that the current definition matches what we want
        if !(current_def < Typelib::CompoundType) || current_def.fields.size != type.fields.size
            raise "#{type.full_name}_m is already defined, but does not match the expected definition. Did you define it yourself ?"
        else
            type_fields    = type.fields.dup
            current_fields = current_def.fields.dup

            while !type_fields.empty?
                expected = type_fields.first
                current  = current_fields.first

                if expected[0] != current[0] ||
                    !expected[1].opaque? && expected[1] != current[1]
                    raise "#{type.full_name}_m is already defined, but does not match the expected definition. Did you define it yourself ?"
                elsif expected[1].opaque?
                    if typekit.intermediate_type_for(field_type) != current[1]
                        raise "#{type.full_name}_m is already defined, but does not match the expected definition. Did you define it yourself ?"
                    end
                end

                type_fields.pop
                current_fields.pop
            end
        end
        next
    end %>

    <%=
    code = Generation.adapt_namespace('/', type.namespace)
    namespace = type.namespace
    did_something = true
    code %>
    <%= type.to_m_type(typekit) %>
    <%= Generation.adapt_namespace(namespace, '/') %>
<% end %>

<% if !did_something
     throw :nothing_to_define
   end %>

