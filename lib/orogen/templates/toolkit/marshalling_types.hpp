<% namespace = '/'
   did_something = false %>

<% registry.each_type do |type|
    next if type.opaque?
    next unless contains_opaques?(type)

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
                    if component.find_type(opaque_specification(field_type).intermediate) != current[1]
                        raise "#{type.full_name}_m is already defined, but does not match the expected definition. Did you define it yourself ?"
                    end
                end

                type_fields.pop
                current_fields.pop
            end
        end
        next
    end %>
<%= code = Generation.adapt_namespace(namespace, type.namespace)
    namespace = type.namespace
    did_something = true
    code %>
struct <%= type.basename %>_m
{
<% type.each_field do |field_name, field_type|
    if field_type.opaque? %>
        <%= component.find_type(opaque_specification(field_type).intermediate).cxx_name %> <%= field_name %>;
    <% else %>
        <%= field_type.cxx_name %> <%= field_name %>;
    <% end %>
<% end %>
};
<% end %>

<%= Generation.adapt_namespace(namespace, '/') %>

<% if !did_something
     throw :nothing_to_define
   end %>

