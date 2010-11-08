module Typelib
    class Type
        def self.to_stream(typekit, result, indent)
            STDERR.puts "to_ostream not implemented for #{name}"
            result
        end
    end

    class ContainerType
        def self.to_stream(typekit, result, indent)
            collection_name, element_type = container_kind, deference.name
            element_type = registry.build(element_type)

            result << indent << "io << \"[ \";\n"

            first_field = true
            allocate_index do |element_idx|
                result << <<-EOT
#{indent}bool first_field = true;
#{indent}for(#{cxx_name}::const_iterator it = value.begin(); it != value.end(); ++it)
#{indent}{
#{indent}    if (!first_field)
#{indent}         io << ", ";
#{indent}    first_field = false;
                EOT

                if element_type.inlines_code?
                    result << "#{indent}    io << *it;\n"
                elsif element_type < ArrayType
                    result << "#{indent}    toStream(\"\", reinterpret_cast<#{element_type.deference.cxx_name} const*>(*it), #{element_size.length}, io);\n"
                else
                    result << "#{indent}    toStream(\"\", *it, io);\n"
                end
                result << "#{indent}}\n"
	    end
            result << indent << "io << \" ]\";\n"
        end

    end

    class EnumType
        def self.to_stream(typekit, result, indent)
            to_string(typekit, result, indent)
            result << "#{indent}io << enum_name;\n";
            result
        end
    end

    class CompoundType
        def self.to_stream(typekit, result, indent)
            result << indent << "io << \"{ \";\n"

            first_field = true
	    each_field do |field_name, field_type|
                unless first_field
                    result << "#{indent}  io << \", \";\n";
                end

                first_field = false
                result << "#{indent}  io << basename << \".#{field_name} = \";\n";
                if field_type.inlines_code?
                    result << "#{indent}  io << value.#{field_name};\n"
                elsif field_type < ArrayType
                    result << "#{indent}  toStream(basename + \".#{field_name}\", reinterpret_cast<#{field_type.deference.cxx_name} const*>(value.#{field_name}), #{field_type.length}, io);\n"
                else
                    result << "#{indent}  toStream(basename + \".#{field_name}\", value.#{field_name}, io);\n"
                end
	    end
            result << indent << "io << \" }\";\n"
        end
    end
    class ArrayType
        def self.to_stream(typekit, result, indent)
            element_type = registry.build(deference.name)

            result << indent << "io << \"[ \";\n"

            first_field = true
            allocate_index do |i|
                result << <<-EOT
#{indent}bool first_field = true;
#{indent}for(int #{i} = 0; #{i} < length; ++#{i})
#{indent}{
#{indent}    if (!first_field)
#{indent}         io << ", ";
#{indent}    first_field = false;
                EOT

                if element_type.inlines_code?
                    result << "#{indent}    io << value[#{i}];\n"
                elsif element_type < ArrayType
                    result << "#{indent}    toStream(\"\", reinterpret_cast<#{element_type.deference.cxx_name} const*>(value[#{i}]), #{element_type.length}, io);\n"
                else
                    result << "#{indent}    toStream(\"\", value[#{i}], io);\n"
                end
                result << "#{indent}}\n"
	    end
            result << indent << "io << \" ]\";\n"
        end
    end
end

