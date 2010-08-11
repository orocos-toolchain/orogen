module Typelib
    class Type
	def self.to_property_bag(toolkit, result, indent = "    ")
            STDERR.puts "to_property_bag not implemented for #{name}"
            result
        end
	def self.from_property_bag(toolkit, result, indent = "    ")
            STDERR.puts "from_property_bag not implemeted for #{name}"
            result
	end

        def self.inline_fromPropertyBag(toolkit, result, varname, property_name, indent)
            orocos_type = registry.base_rtt_type_for(self).cxx_name
            result << indent << "#{varname} = bag.getProperty<#{orocos_type}>(#{property_name})->get();\n"
        end
        def self.inline_toPropertyBag(toolkit, result, varname, property_name, indent)
            orocos_type = registry.base_rtt_type_for(self).cxx_name
            result << indent << "target_bag.add( new Property<#{orocos_type}>(#{property_name}, \"\", #{varname}) );\n"
        end
    end

    class EnumType
        def self.to_property_bag(toolkit, result, indent)
            to_string(toolkit, result, indent)
            result << "#{indent}target_bag.add( new Property<std::string>(basename, \"\", enum_name) );\n"
            result
        end

        def self.from_property_bag(toolkit, result, indent)
            result << indent << "std::string enum_name;\n"
            result << indent << "enum_name = bag.getProperty<std::string>(basename)->get();\n"
            first_key = true
            keys.each do |name, _|
                result << indent << "#{'else ' if !first_key}if(enum_name == \"#{name}\")\n"
                result << indent << "   value = #{namespace('::')}#{name};\n"
                first_key = false
            end
            result << <<-EOT
else
{
    RTT::log(RTT::Error) << "invalid value " << enum_name << " for enum #{cxx_name}" << RTT::endlog();
    return false;
}
            EOT
            result
        end
    end

    class CompoundType
	def self.to_property_bag(toolkit, result, indent)
            each_field do |field_name, field_type|
                if field_type.inlines_code?
                    field_type.inline_toPropertyBag(toolkit, result, "value.#{field_name}", "basename + \".#{field_name}\"", indent)
                elsif field_type < ArrayType
                    result << "#{indent}if (!toPropertyBag(basename + \".#{field_name}\", reinterpret_cast<#{field_type.deference.cxx_name} const*>(value.#{field_name}), #{field_type.length}, target_bag)) return false;\n";
                else
                    result << "#{indent}if (!toPropertyBag(basename + \".#{field_name}\", value.#{field_name}, target_bag)) return false;\n";
                end
            end
	    result
	end

	def self.from_property_bag(toolkit, result, indent)
            each_field do |field_name, field_type|
                if field_type.inlines_code?
                    field_type.inline_fromPropertyBag(toolkit, result, "value.#{field_name}", "basename + \".#{field_name}\"", indent)
                elsif field_type < ArrayType
                    result << "#{indent}if (!fromPropertyBag(basename + \".#{field_name}\", reinterpret_cast<#{field_type.deference.cxx_name}*>(value.#{field_name}), #{field_type.length}, bag)) return false;\n";
                else
                    result << "#{indent}if (!fromPropertyBag(basename + \".#{field_name}\", value.#{field_name}, bag)) return false;\n";
                end
            end
	    result
	end
    end

    class ArrayType
	def self.to_property_bag(toolkit, result, indent)
            element_type = registry.build(deference.name)

            allocate_index do |i|
                idx_expr = "basename + \"[\" + boost::lexical_cast<std::string>(#{i}) + \"]\""
                result << <<-EOT
#{indent}for(int #{i} = 0; #{i} < length; ++#{i})
#{indent}{
                EOT

                if element_type.inlines_code?
                    element_type.inline_toPropertyBag(toolkit, result, "value[#{i}]", idx_expr, indent + "    ")
                elsif element_type < ArrayType
                    result << "#{indent}if (!toPropertyBag(#{idx_expr}, reinterpret_cast<#{field_type.deference.cxx_name} const*>(value[#{i}]), #{element_type.length}, target_bag)) return false;\n";
                else
                    result << "#{indent}if (!toPropertyBag(#{idx_expr}, value[#{i}], target_bag)) return false;\n";
                end

                result << "#{indent}}\n";
                result << "#{indent}target_bag.add( new Property<int>(basename + \".size\", \"\", length) );\n"
            end
	    result
	end

	def self.from_property_bag(toolkit, result, indent = "    ")
            element_type = registry.build(deference.name)

            allocate_index do |i|
                idx_expr = "basename + \"[\" + boost::lexical_cast<std::string>(#{i}) + \"]\""
                result << "#{indent}for (int #{i} = 0; #{i} < length; ++#{i})\n"
                result << "#{indent}{\n"
                if element_type.inlines_code?
                    element_type.inline_fromPropertyBag(toolkit, result, "value[#{i}]", idx_expr, indent + "    ")
                elsif element_type < ArrayType
                    result << "#{indent}    if (!fromPropertyBag(#{idx_expr}, reinterpret_cast<#{field_type.deference.cxx_name}*>(value[#{i}]), #{element_type.length}, bag)) return false;\n";
                else
                    result << "#{indent}    if (!fromPropertyBag(#{idx_expr}, value[#{i}], bag)) return false;\n";
                end
                result << "#{indent}}\n"
            end
            result
        end
    end

    class ContainerType
	def self.to_property_bag(toolkit, result, indent)
            collection_name, element_type = container_kind, deference.name
            element_type = registry.build(element_type)

            allocate_index do |element_idx|
                idx_expr = "basename + \"[\" + boost::lexical_cast<std::string>(#{element_idx}) + \"]\""
                result << <<-EOT
#{indent}size_t #{element_idx} = 0;
#{indent}for(#{cxx_name}::const_iterator it = value.begin(); it != value.end(); ++it, ++#{element_idx})
#{indent}{
                EOT

                if element_type.inlines_code?
                    element_type.inline_toPropertyBag(toolkit, result, "(*it)", idx_expr, indent + "    ")
                elsif element_type < ArrayType
                    result << indent << "if (!toPropertyBag(#{idx_expr}, reinterpret_cast<#{field_type.deference.cxx_name} const*>(*it), #{element_type.length}, target_bag)) return false;\n";
                else
                    result << indent << "if (!toPropertyBag(#{idx_expr}, *it, target_bag)) return false;\n";
                end

                result << "#{indent}}\n";
                result << "#{indent}target_bag.add( new Property<int>(basename + \".size\", \"\", #{element_idx}) );\n"
            end
	    result
	end
	def self.from_property_bag(toolkit, result, indent = "    ")
             collection_name, element_type = container_kind, deference.name
             element_type = registry.build(element_type)

             if container_kind != "/std/vector"
                 raise NotImplementedError, "from_property_bag is not implemented for other containers than std::vector, got #{container_kind}"
             end

             allocate_index do |i|
                 idx_expr = "basename + \"[\" + boost::lexical_cast<std::string>(#{i}) + \"]\""
                 result << "#{indent}size_t size_#{i} = bag.getProperty<int>(basename + \".size\")->get();\n"
                 result << "#{indent}value.resize(size_#{i});\n"
                 result << "#{indent}for (size_t #{i} = 0; #{i} < size_#{i}; ++#{i})\n"
                 result << "#{indent}{\n"
                 if element_type.inlines_code?
                     element_type.inline_fromPropertyBag(toolkit, result, "value[#{i}]", idx_expr, indent + "    ")
                 elsif element_type < ArrayType
                     result << "#{indent}    fromPropertyBag(#{idx_expr}, value[#{i}], #{element_type.length}, bag);\n";
                 else
                     result << "#{indent}    fromPropertyBag(#{idx_expr}, value[#{i}], bag);\n";
                 end
                 result << "#{indent}}\n"
             end
	     result
	end
    end
    specialize_model '/std/string' do
        def to_property_bag(toolkit, result, indent)
            inline_toPropertyBag(toolkit, result, "value", "basename", indent);
        end
        def from_property_bag(toolkit, result, indent)
            inline_fromPropertyBag(toolkit, result, "value", "basename", indent);
        end
    end
end

