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
    target_typename = typekit.intermediate_type_name_for(type)
    current_def = begin
                      typekit.find_type(target_typename)
                  rescue Typelib::NotFound
                  end

    if !generate_all_marshalling_types && current_def
        expected_type = type
        while current_def.respond_to?(:deference)
            current_def = current_def.deference
            if !expected_type.respond_to?(:deference)
                raise "#{current_def.name} is already defined, but does not match the expected definition. Did you define it yourself ?"
            end
            expected_type = expected_type.deference
        end

        if current_def < Typelib::CompoundType
            if expected_type.opaque?
                # nothing to do
            elsif !(expected_type < Typelib::CompoundType)
                raise "#{current_def.name} is already defined, but does not match the expected definition. Did you define it yourself ?"
            elsif current_def.fields.size != expected_type.fields.size
                raise "#{current_def.name} is already defined, but does not match the expected definition. Did you define it yourself ?"
            else
                type_fields    = expected_type.fields.dup
                current_fields = current_def.fields.dup

                while !type_fields.empty?
                    expected = type_fields.first
                    current  = current_fields.first

                    if expected[0] != current[0] ||
                        !expected[1].opaque? && expected[1] != current[1]
                        raise "#{current_def.name} is already defined, but does not match the expected definition. Did you define it yourself ?"
                    elsif expected[1].opaque?
                        if typekit.intermediate_type_for(expected[1]) != current[1]
                            raise "#{current_def.name} is already defined, but does not match the expected definition. Did you define it yourself ?"
                        end
                    end

                    type_fields.pop
                    current_fields.pop
                end
            end
        end
        next
    end %>

    <% target_namespace = Typelib.namespace(target_typename)
       target_basename  = Typelib.basename(target_typename)
        target_basename.gsub!('/', '::')
    did_something = true %>
    <%= Generation.adapt_namespace('/', target_namespace) %>
    <%= type.to_m_type(target_basename, typekit) %>
    <%= Generation.adapt_namespace(target_namespace, '/') %>
<% end %>

<% if !did_something
     throw :nothing_to_define
   end %>

