module Typelib
    class Type
        def self.corba_name(plain_name = false)
            if inlines_code?
                normalize_cxxname(basename)
            elsif contains_opaques?
                "orogen#{namespace('::')}Corba::#{normalize_cxxname(basename).gsub(/[^\w]/, '_')}_m"
            else
                "orogen#{namespace('::')}Corba::#{normalize_cxxname(basename).gsub(/[^\w]/, '_')}"
            end
        end

        def self.corba_arg_type; "#{corba_name} const&" end
        def self.corba_ref_type; "#{corba_name}&" end

        def self.to_corba(toolkit, result, *args)
            STDERR.puts "to_corba not implemented for #{name}"
            result
        end
        def self.from_corba(toolkit, result, *args)
            STDERR.puts "from_corba not implemented for #{name}"
            result
        end

        def self.inline_fromCorba(result, value, indent)
            "#{indent}#{result} = #{value};\n"
        end
        def self.inline_toCorba(result, value, indent)
            "#{indent}#{result} = #{value};\n"
        end
    end

    class NumericType
        def self.corba_name(plain_name = false)
	    if integer?
		if name == "/bool"
		    "CORBA::Boolean"
                elsif size == 1
                    "CORBA::Octet"
                elsif size == 2
                    "CORBA::#{'U' if unsigned?}Short"
                elsif size == 4
                    "CORBA::#{'U' if unsigned?}Long"
                elsif size == 8
                    "CORBA::#{'U' if unsigned?}LongLong"
                else
                    raise "unexpected integer size #{size}"
		end
	    else
                if size == 4
                    "CORBA::Float"
                elsif size == 8
                    "CORBA::Double"
                else
                    raise "unexpected floating-point size #{size}"
                end
	    end
        end
    end

    specialize_model '/std/string' do
        def inlines_code?; true end
        def corba_name(plain_name = false); "char" end
        def corba_arg_type; "char const*" end
        def corba_ref_type; "CORBA::String_member&" end

        def inline_toCorba(result, value, indent)
            "#{indent}#{result} = #{value}.c_str();"
        end
    end

    class ContainerType
        def self.corba_name(plain_name = false)
            if plain_name
                return super
            end

            if deference.corba_name == "CORBA::Octet"
                "_CORBA_Unbounded_Sequence_Octet"
            else
                "_CORBA_Unbounded_Sequence< #{deference.corba_name} >"
            end
        end
        def self.corba_arg_type; "#{corba_name} const&" end
        def self.corba_ref_type; "#{corba_name}&" end

        def self.to_corba(toolkit, result, indent)
            collection_name, element_type = container_kind, deference.name
            element_type = registry.build(element_type)

            result << "#{indent}corba.length(value.size());\n"
            # Special case for array of bytes, we can do a simple memcpy
            # (maybe extend that later)
            if collection_name == "/std/vector" && 
                element_type < NumericType &&
                element_type.integer? &&
                element_type.size == 1

                result << "if (!value.empty()) #{indent}memcpy(&corba[0], &value[0], value.size());"
            else
                allocate_index do |element_idx|
                    result << <<-EOT
    #{indent}size_t #{element_idx} = 0;
    #{indent}for(#{cxx_name}::const_iterator it = value.begin(); it != value.end(); ++it, ++#{element_idx})
    #{indent}{
                    EOT

                    if element_type.inlines_code?
                        result << element_type.inline_toCorba("corba[#{element_idx}]", "(*it)", indent)
                    elsif element_type < ArrayType
                        result << indent << "    toCORBA(corba[#{element_idx}], reinterpret_cast<#{element_type.deference.cxx_name} const*>(*it), #{element_type.length});\n";
                    else
                        result << indent << "    toCORBA(corba[#{element_idx}], *it);\n";
                    end

                    result << "#{indent}}\n";
                end
            end
	    result
        end
        def self.from_corba(toolkit, result, indent)
            collection_name, element_type = container_kind, deference.name
            element_type = registry.build(element_type)

            if container_kind != "/std/vector"
                raise NotImplementedError, "from_corba is not implemented for other containers than std::vector, got #{container_kind}"
            end

            allocate_index do |element_idx|
                result << "#{indent}size_t const size_#{element_idx} = corba.length();\n"
                result << "#{indent}value.resize(size_#{element_idx});\n"

                if collection_name == "/std/vector" && 
                    element_type < NumericType &&
                    element_type.integer? &&
                    element_type.size == 1

                    result << "if (!value.empty()) #{indent}memcpy(&value[0], &corba[0], size_#{element_idx});"
                else
                    result << <<-EOT
    #{indent}for(size_t #{element_idx} = 0; #{element_idx} < size_#{element_idx}; ++#{element_idx})
    #{indent}{
                    EOT

                    if element_type.inlines_code?
                        result << element_type.inline_fromCorba("value[#{element_idx}]", "corba[#{element_idx}]", indent)
                    elsif element_type < ArrayType
                        result << "#{indent}    fromCORBA(reinterpret_cast<#{element_type.deference.cxx_name}*>(value[#{element_idx}]), #{element_type.length}, corba[#{element_idx}]);\n";
                    else
                        result << "#{indent}    fromCORBA(value[#{element_idx}], corba[#{element_idx}]);\n";
                    end

                    result << "#{indent}}\n";
                end
            end
	    result
        end
    end

    class EnumType
	def self.to_corba(toolkit, result, indent)
            seen_values = Set.new
            namespace = self.namespace('::')
            result << indent << "switch(value) {\n"
            keys.each do |name, value|
                next if seen_values.include?(value)
                seen_values << value

                result << indent << "  case #{namespace}#{name}:\n"
                result << indent << "    corba = orogen#{namespace}Corba::#{name};\n"
                result << indent << "    break;\n"
            end
            result << indent << "}\n"
	end
	def self.from_corba(toolkit, result, indent)
            seen_values = Set.new
            namespace = self.namespace('::')
            result << indent << "switch(corba) {\n"
            keys.each do |name, value|
                next if seen_values.include?(value)
                seen_values << value

                result << indent << "  case orogen#{namespace}Corba::#{name}:\n"
                result << indent << "    value = #{namespace}#{name};\n"
                result << indent << "    break;\n"
            end
            result << indent << "}\n"
	end
    end

    class CompoundType
        def self.to_corba(toolkit, result, indent)
            code_copy(toolkit, result, indent, "corba", "value", "toCORBA", true) do |field_name, field_type|
                if field_type.inlines_code?
                    field_type.inline_toCorba("corba.#{field_name}", "value.#{field_name}", indent)
                end
            end
        end
        def self.from_corba(toolkit, result, indent)
            code_copy(toolkit, result, indent, "value", "corba", "fromCORBA", true) do |field_name, field_type|
                if field_type.inlines_code?
                    field_type.inline_fromCorba("value.#{field_name}", "corba.#{field_name}", indent)
                end
            end
        end
    end
    class ArrayType
        def self.corba_arg_type; "#{deference.corba_name} const*" end
        def self.corba_ref_type; "#{deference.corba_name}*" end

        def self.to_corba(toolkit, result, indent)
            element_type = deference.name
            element_type = registry.build(element_type)

            # Special case for array of numerics, we can do a simple memcpy
            if  element_type < NumericType 
                result << "#{indent}const int array_size = length * sizeof(#{element_type.cxx_name});\n"
                result << "#{indent}memcpy(corba, value, array_size);"
            else
                code_copy(toolkit, result, indent, "corba", "value", "toCORBA") do |type, _|
                    type.inlines_code?
                end
            end
        end
        def self.from_corba(toolkit, result, indent)
            element_type = deference.name
            element_type = registry.build(element_type)

            # Special case for array of numerics, we can do a simple memcpy
            if  element_type < NumericType 
                result << "#{indent}const int array_size = length * sizeof(#{element_type.cxx_name});\n"
                result << "#{indent}memcpy(value, corba, array_size);"
            else
                code_copy(toolkit, result, indent, "value", "corba", "fromCORBA") do |type, _|
                    type.inlines_code?
                end
            end
        end
    end
end

