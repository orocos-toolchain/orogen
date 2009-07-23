require 'typelib'
require 'tempfile'
require 'find'
require 'orogen/base'
require 'utilrb/kernel/options'
require 'nokogiri'

module Typelib
    class Type
        def self.normalize_typename(name)
            name.to_str.gsub(/<(\w)/) { "</#{$1}" }
        end

        def self.normalize_cxxname(name)
            name.gsub("<::", "<").gsub('>>', '> >')
        end

        def self.cxx_name(fullname = true)
            normalize_cxxname(full_name('::', fullname))
        end
        def self.cxx_basename
            normalize_cxxname(basename('::'))
        end
        def self.cxx_namespace
            namespace('::')
        end

        def self.corba_name
            if inlines_code?
                normalize_cxxname(basename)
            else
                "#{namespace('::')}Corba::#{normalize_cxxname(basename).gsub(/[^\w]/, '_')}"
            end
        end

        def self.arg_type; "#{cxx_name} const&" end
        def self.ref_type; "#{cxx_name}&" end
        def self.corba_arg_type; "#{corba_name} const&" end
        def self.corba_ref_type; "#{corba_name}&" end

        def self.method_name(fullname = true)
            base = if fullname then full_name('_', true)
                   else basename('_')
                   end
            base.gsub(/[<>\[\]]/, '_')
        end

	def self.contains_int64?; false end
        def self.contains_opaques?; opaque? end

        @@index_var_stack = Array.new
        def self.index_var_stack; @@index_var_stack end
        def self.allocate_index
            index_var_stack.push "i#{index_var_stack.size}"
            yield(index_var_stack.last)
        ensure
            index_var_stack.pop
        end

        def self.inlines_code?
            superclass.eql?(NumericType) || (opaque? && (name == "/std/string" || name == "string"))
        end

	def self.to_property_bag(toolkit, result, indent = "    ")
            STDERR.puts "to_property_bag not implemented for #{name}"
            result
        end
        def self.to_corba(toolkit, result, *args)
            STDERR.puts "to_corba not implemented for #{name}"
            result
        end
        def self.from_corba(toolkit, result, *args)
            STDERR.puts "from_corba not implemented for #{name}"
            result
        end
        def self.to_stream(toolkit, result, indent)
            STDERR.puts "to_ostream not implemented for #{name}"
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
        def self.inline_toCORBA(toolkit, result, corbavar, cxxvar, indent)
            result << indent << "#{corbavar} = #{cxxvar};\n";
        end
        def self.inline_fromCORBA(toolkit, result, corbavar, cxxvar, indent)
            result << indent << "#{cxxvar} = #{corbavar};\n";
        end

        # Generate the code needed to initialize a C++ variable from an Orocos
        # PropertyBag.
        #
        # +toolkit+ is the toolkit on which we act. +result+ is the generated
        # code as a string. +var+ is the name of the variable that holds the
        # value which should be decomposed. +property_name+ is the name the
        # property should have in the property bag and finally +indent+ is the
        # indentation of the code (needed to have a readable code in the end).
	# def self.from_orocos_decomposition(toolkit, result, path, indent = "    ", property_name = nil)
        #     property_name ||= "\"#{path[1..-1]}\""
        #     if opaque?
        #         result << indent << "{ PropertyBag const& inner_bag = bag.getProperty<PropertyBag>(#{property_name})->value();\n"
        #         result << indent << "   #{cxx_name}TypeInfo::doCompose(inner_bag, out#{path});\n"
        #         result << indent << "}"
        #     else
        #         orocos_type = registry.base_rtt_type_for(self).cxx_name
        #         result << indent << "out#{path} = bag.getProperty<#{orocos_type}>(#{property_name})->get();"
        #     end
	# end

        # Generate the code needed to decompose a give C++ type into a
        # propertybag (for further marshalling by Orocos).
        #
        # +toolkit+ is the toolkit on which we act. +result+ is the generated
        # code as a string. +var+ is the name of the variable that holds the
        # value which should be decomposed. +property_name+ is the name the
        # property should have in the property bag and finally +indent+ is the
        # indentation of the code (needed to have a readable code in the end).
	# def self.to_orocos_decomposition(toolkit, result, path, indent = "    ", property_name = nil)
        #     property_name ||= "\"#{path[1..-1]}\""
        #     if opaque?
        #         result << indent << "{ PropertyBag inner_bag(\"#{full_name}\");\n"
        #         result << indent << "   #{cxx_namespace}#{method_name(false)}TypeInfo::doDecompose(value#{path}, inner_bag);\n"
        #         result << indent << "   Property<PropertyBag>* temp_property = new Property<PropertyBag>(#{property_name}, \"\", inner_bag);\n"
        #         result << indent << "   target_bag.add(temp_property);\n"
        #         result << indent << "}"
        #     else
        #         orocos_type = registry.base_rtt_type_for(self).cxx_name
        #         result << indent << "target_bag.add( new Property<#{orocos_type}>(#{property_name}, \"\", value#{path}) );"
        #     end
	# end

	# def self.code_to_corba(toolkit, result, path = "", indent = "    ")
        #     if opaque?
        #         result << indent << "result#{path} = *AnyConversion< #{cxx_name} >::createAny(value#{path});"
        #     else
        #         result << indent << "result#{path} = value#{path};"
        #     end
	# end
	# def self.code_from_corba(toolkit, result, path = "", indent = "    ")
        #     if opaque?
        #         result << indent << "AnyConversion< #{cxx_name} >::updateFromAny(value#{path}, result#{path});"
        #     else
        #         result << indent << "result#{path} = value#{path};"
        #     end
	# end
        # def self.to_ostream(toolkit, result, path, indent)
        #     if opaque?
        #         result << indent << "io << data#{path};"
        #     else
        #         orocos_type = registry.base_rtt_type_for(self).cxx_name
        #         property_name = path[1..-1]
        #         result << indent << "io << static_cast<#{orocos_type}>(data#{path});"
        #     end
        # end
    end

    class NumericType
	def self.cxx_name
	    if integer?
		if name == "/bool"
		    "bool"
                elsif name == "/char"
                    "char"
                elsif name == "/unsigned char"
                    "unsigned char"
		else
		    "boost::#{'u' if unsigned?}int#{size * 8}_t"
		end
	    else
		basename
	    end
	end
        def self.corba_name
	    if integer?
		if name == "/bool"
		    "CORBA::Boolean"
                elsif size == 1
                    "CORBA::Octet"
		else
		    "boost::#{'u' if unsigned?}int#{size * 8}_t"
		end
	    else
		basename
	    end
        end

	def self.contains_int64?; size == 8 && integer?  end
    end

    class ContainerType
	def self.contains_int64?; deference.contains_int64?  end
        def self.contains_opaques?; deference.contains_opaques? end

        def self.cxx_name
            kind = container_kind.
                gsub('/', '::').
                gsub(/^::/, '')
            normalize_cxxname(kind + "< " + deference.cxx_name + " >")
        end
        def self.corba_generic_name
            if deference.corba_name == "CORBA::Octet"
                "_CORBA_Unbounded_Sequence_Octet"
            else
                "_CORBA_Unbounded_Sequence< #{deference.corba_name} >"
            end
        end
        def self.corba_arg_type; "#{corba_generic_name} const&" end
        def self.corba_ref_type; "#{corba_generic_name}&" end

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
                else
                    result << indent << "if (!toPropertyBag(#{idx_expr}, *it, target_bag)) return false;\n";
                end

                result << "#{indent}}\n";
                result << "#{indent}target_bag.add( new Property<int>(basename + \".size\", \"\", #{element_idx}) );\n"
            end
	    result
	end

        def self.to_corba(toolkit, result, indent)
            collection_name, element_type = container_kind, deference.name
            element_type = registry.build(element_type)

            result << "#{indent}corba.length(value.size());\n"
            allocate_index do |element_idx|
                result << <<-EOT
#{indent}size_t #{element_idx} = 0;
#{indent}for(#{cxx_name}::const_iterator it = value.begin(); it != value.end(); ++it, ++#{element_idx})
#{indent}{
                EOT

                if element_type.inlines_code?
                    element_type.inline_toCORBA(toolkit, result, "corba[#{element_idx}]", "(*it)", indent + "    ")
                else
                    result << indent << "    toCORBA(corba[#{element_idx}], *it);\n";
                end

                result << "#{indent}}\n";
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
                result << <<-EOT
#{indent}for(size_t #{element_idx} = 0; #{element_idx} < size_#{element_idx}; ++#{element_idx})
#{indent}{
                EOT

                if element_type.inlines_code?
                    element_type.inline_fromCORBA(toolkit, result, "corba[#{element_idx}]", "value[#{element_idx}]", indent + "    ")
                else
                    result << indent << "    fromCORBA(corba[#{element_idx}], value[#{element_idx}]);\n";
                end

                result << "#{indent}}\n";
            end
	    result
        end
        def self.to_stream(toolkit, result, indent)
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
                else
                    result << "#{indent}    toStream(\"\", *it, io);\n"
                end
                result << "#{indent}}\n"
	    end
            result << indent << "io << \" ]\";\n"
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
                 else
                     result << "#{indent}    fromPropertyBag(#{idx_expr}, value[#{i}], bag);\n";
                 end
                 result << "#{indent}}\n"
             end
	     result
	end
    end

    class EnumType
        def self.to_string(toolkit, result, indent)
            result << indent << "std::string enum_name;\n"
            result << indent << "switch(value) {\n"
            keys.each do |name, _|
                result << "#{indent}    case #{namespace('::')}#{name}:\n"
                result << "#{indent}      enum_name = \"#{name}\";\n"
                result << "#{indent}      break;\n"
            end

            result << <<-EOT
#{indent}    default:
#{indent}        RTT::log(RTT::Error) << "invalid value " << (int)value << " for enum #{cxx_name}" << RTT::endlog();
#{indent}        return false;
#{indent}}
            EOT
            result
        end

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

        def self.to_stream(toolkit, result, indent)
            to_string(toolkit, result, indent)
            result << "#{indent}io << enum_name;\n";
            result
        end

	def self.to_corba(toolkit, result, indent)
            namespace = self.namespace('::')
            result << indent << "switch(value) {\n"
            keys.each do |name, _|
                result << indent << "  case #{namespace}#{name}:\n"
                result << indent << "    corba = #{namespace}Corba::#{name};\n"
                result << indent << "    break;\n"
            end
            result << indent << "}\n"
	end
	def self.from_corba(toolkit, result, indent)
            namespace = self.namespace('::')
            result << indent << "switch(corba) {\n"
            keys.each do |name, _|
                result << indent << "  case #{namespace}Corba::#{name}:\n"
                result << indent << "    value = #{namespace}#{name};\n"
                result << indent << "    break;\n"
            end
            result << indent << "}\n"
	end
    end

    class CompoundType
	def self.contains_int64?; enum_for(:each_field).any? { |_, type| type.contains_int64? } end
	def self.contains_opaques?; enum_for(:each_field).any? { |_, type| type.contains_opaques? } end
	def self.to_property_bag(toolkit, result, indent)
            each_field do |field_name, field_type|
                if field_type.inlines_code?
                    field_type.inline_toPropertyBag(toolkit, result, "value.#{field_name}", "basename + \".#{field_name}\"", indent)
                else
                    result << "#{indent}if (!toPropertyBag(basename + \".#{field_name}\", value.#{field_name}, target_bag)) return false;\n";
                end
            end
	    result
	end
        def self.to_corba(toolkit, result, indent)
            each_field do |field_name, field_type|
                if field_type.inlines_code?
                    field_type.inline_toCORBA(toolkit, result, "corba.#{field_name}", "value.#{field_name}", indent)
                else
                    result << "#{indent}if (!toCORBA(corba.#{field_name}, value.#{field_name})) return false;\n";
                end
            end
	    result
        end
        def self.from_corba(toolkit, result, indent)
            each_field do |field_name, field_type|
                if field_type.inlines_code?
                    field_type.inline_fromCORBA(toolkit, result, "corba.#{field_name}", "value.#{field_name}", indent)
                else
                    result << "#{indent}if (!fromCORBA(corba.#{field_name}, value.#{field_name})) return false;\n";
                end
            end
	    result
        end

	def self.from_property_bag(toolkit, result, indent)
            each_field do |field_name, field_type|
                if field_type.inlines_code?
                    field_type.inline_fromPropertyBag(toolkit, result, "value.#{field_name}", "basename + \".#{field_name}\"", indent)
                else
                    result << "#{indent}if (!fromPropertyBag(basename + \".#{field_name}\", value.#{field_name}, bag)) return false;\n";
                end
            end
	    result
	end

        def self.to_stream(toolkit, result, indent)
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
                else
                    result << "#{indent}  toStream(basename + \".#{field_name}\", value.#{field_name}, io);\n"
                end
	    end
            result << indent << "io << \" }\";\n"
        end
    end
    class ArrayType
	def self.contains_int64?; deference.contains_int64?  end
        def self.contains_opaques?; deference.contains_opaques? end

        def self.arg_type; "#{deference.cxx_name} const*" end
        def self.ref_type; "#{deference.cxx_name}*" end
        def self.corba_arg_type; "#{deference.corba_name} const*" end
        def self.corba_ref_type; "#{deference.corba_name}*" end

	def self.to_property_bag(toolkit, result, indent)
            element_type = registry.build(deference.name)

            allocate_index do |i|
                idx_expr = "basename + \"[\" + boost::lexical_cast<std::string>(#{i}) + \"]\""
                result << <<-EOT
#{indent}for(size_t #{i} = 0; #{i} < #{length}; ++#{i})
#{indent}{
                EOT

                if element_type.inlines_code?
                    element_type.inline_toPropertyBag(toolkit, result, "value[#{i}]", idx_expr, indent + "    ")
                else
                    result << "#{indent}if (!toPropertyBag(#{idx_expr}, value[#{i}], target_bag)) return false;\n";
                end

                result << "#{indent}}\n";
                result << "#{indent}target_bag.add( new Property<int>(basename + \".size\", \"\", #{length}) );\n"
            end
	    result
	end

        def self.to_corba(toolkit, result, indent)
            element_type = registry.build(deference.name)

            allocate_index do |i|
                result << <<-EOT
#{indent}for(size_t #{i} = 0; #{i} < #{length}; ++#{i})
#{indent}{
                EOT

                if element_type.inlines_code?
                    element_type.inline_toCORBA(toolkit, result, "corba[#{i}]", "value[#{i}]", indent + "    ")
                else
                    result << indent << "    if (!toCORBA(corba[#{i}], value[#{i}])) return false;\n";
                end

                result << "#{indent}}\n";
            end
	    result
        end
        def self.from_corba(toolkit, result, indent)
            element_type = registry.build(deference.name)

            allocate_index do |i|
                result << <<-EOT
#{indent}for(size_t #{i} = 0; #{i} < #{length}; ++#{i})
#{indent}{
                EOT

                if element_type.inlines_code?
                    element_type.inline_fromCORBA(toolkit, result, "corba[#{i}]", "value[#{i}]", indent + "    ")
                else
                    result << indent << "    if (!fromCORBA(corba[#{i}], value[#{i}])) return false;\n";
                end

                result << "#{indent}}\n";
            end
	    result
        end
        def self.to_stream(toolkit, result, indent)
            element_type = registry.build(deference.name)

            result << indent << "io << \"[ \";\n"

            first_field = true
            allocate_index do |i|
                result << <<-EOT
#{indent}bool first_field = true;
#{indent}for(size_t #{i} = 0; #{i} < #{length}; ++#{i})
#{indent}{
#{indent}    if (!first_field)
#{indent}         io << ", ";
#{indent}    first_field = false;
                EOT

                if element_type.inlines_code?
                    result << "#{indent}    io << value[#{i}];\n"
                else
                    result << "#{indent}    toStream(\"\", value[#{i}], io);\n"
                end
                result << "#{indent}}\n"
	    end
            result << indent << "io << \" ]\";\n"
        end

	def self.from_property_bag(toolkit, result, indent = "    ")
            element_type = registry.build(deference.name)

            allocate_index do |i|
                idx_expr = "basename + \"[\" + boost::lexical_cast<std::string>(#{i}) + \"]\""
                result << "#{indent}for (size_t #{i} = 0; #{i} < #{length}; ++#{i})\n"
                result << "#{indent}{\n"
                if element_type.inlines_code?
                    element_type.inline_fromPropertyBag(toolkit, result, "value[#{i}]", idx_expr, indent + "    ")
                else
                    result << "#{indent}    if (!fromPropertyBag(#{idx_expr}, value[#{i}], bag)) return false;\n";
                end
                result << "#{indent}}\n"
            end
            result
        end
    end

    class Registry
        def self.base_rtt_type?(type)
            if type.name == "/std/string"
                return true
            elsif !(type <= Typelib::NumericType)
                return false
            end

            if type.integer?
                type.name == "/bool" || type.size == 4 || type.size == 8
            else
                true
            end
        end
        def base_rtt_type_for(type)
	    if Registry.base_rtt_type?(type)
		type
            elsif type < Typelib::NumericType
                # 64 bit types are directly defined in RTT
                if type.unsigned? then get("/unsigned int")
                else get("/int")
                end
            else
                raise ArgumentError, "no type equivalent for #{type.name} in Orocos"
            end
        end
    end
end

module Orocos
    module Generation
        class OpaqueDefinition
            attr_reader :type
            attr_reader :intermediate
            attr_reader :options
            attr_reader :code_generator

            def initialize(type, intermediate, options, code_generator)
                @type, @intermediate, @options, @code_generator =
                    type, intermediate, options, code_generator
            end
            def includes; options[:includes] end
            def needs_copy?; !!options[:needs_copy] end
            def generate_templates?; !code_generator end
        end

	class Toolkit
	    attr_reader :component
	    attr_reader :imports, :loads
            def external_loads
                loads.find_all do |name|
                    name !~ /#{Regexp.quote(component.base_dir)}/
                end
            end

	    attr_reader :registry
            attr_reader :preloaded_registry
            attr_reader :opaques
            attr_reader :opaque_registry

	    dsl_attribute :blob_threshold do |value|
		value = Integer(value)
		if value == 0; nil
		else value
		end
	    end

	    def find_type(type)
		if type
		    if type.respond_to?(:to_str)
                        type = Typelib::Type.normalize_typename(type)
                        begin
                            registry.build(type)
                        rescue Typelib::NotFound
                            if type =~ /^([^<]+)<(.*)>$/
                                container_name = $1
                                element_name   = $2
                                element_type   = find_type(element_name)
                                if element_type.contains_opaques?
                                    raise ArgumentError, "cannot create a container of #{element_name}: it contains an opaque type or is opaque itself"
                                end
                                component.registry.define_container(container_name,
                                                component.registry.build(element_name))
                                registry.define_container(container_name, element_type)
                            end
                        end
		    elsif type.kind_of?(Class) && type <= Typelib::Type
                        type
                    else
			raise ArgumentError, "expected a type object, got #{type}"
		    end
		end
	    end

            # True if we are generating for Linux
            def linux?;     component.linux? end
            # True if we are generating for Xenomai
            def xenomai?;   component.xenomai? end

	    def initialize(component)
		@component = component

                @internal_dependencies = []
		@corba_enabled      = nil
		@imports, @loads    = [], []
		@registry           = Typelib::Registry.new
		@preloaded_registry = Typelib::Registry.new
		@opaque_registry    = Typelib::Registry.new
                @opaques            = Array.new

		# Load orocos-specific types which cannot be used in the
		# component-defined toolkit but can be used literally in argument
		# lists or property types
		registry.import File.expand_path('orocos.tlb', File.dirname(__FILE__))
	    end

            # Loads the types defined in +file+, with the same constraints as
            # for #load, but will not generate toolkit code for it. This is
            # needed to load system files for which there is a "natural"
            # support in Orocos.
            def preload(file)
                load(file, true)
            end

            # Ask to support a smart-pointer implementation on the given type.
            # For instance, after the call
            #  
            #   smart_ptr "boost::smart_ptr", "int"
            #
            # You will be able to use the following type in your task
            # definitions:
            #
            #   boost::smart_ptr_name<T>
            #
            # The smart pointer class needs to meet the following requirements:
            #  * the memory should be automatically managed (allocated/deallocated)
            #  * the star operator can be used to get a const reference on the
            #    managed value (i.e. <tt>*pointer</tt> returns the object)
            #  * the smart pointer must define a #reset method to set a new
            #    managed memory zone.
            #
            # The Toolkit#ro_ptr and Toolkit#shared_ptr shortcuts are defined
            # for boost::shared_ptr and RTT::ReadOnlyPointer.
            def smart_ptr(name, base_type, options = Hash.new)
                opaque_type("#{name}<#{base_type.name}>", base_type, options.merge(:needs_copy => false)) do |from, into|
                    code = Generation.render_template('toolkit/smart_ptr.cpp', binding)
                end
            end

            # Make the toolkit define the specialization of RTT::ReadOnlyPointer
            # for the given type.
            #
            # See #smart_ptr for more information.
            def ro_ptr(name, options = Hash.new)
                options[:includes] ||= Array.new
                options[:includes] << 'rtt/ReadWritePointer.hpp'
                smart_ptr("/RTT/ReadOnlyPointer", find_type(name), options)
            end

            # Make the toolkit define the specialization of boost::shared_ptr
            # for the given type.
            #
            # See #smart_ptr for more information.
            def shared_ptr(name, options = Hash.new)
                options[:includes] ||= Array.new
                options[:includes] << 'boost/shared_ptr.hpp'
                smart_ptr("/boost/shared_ptr", find_type(name), options)
            end


            # Declare that the user will provide a method which converts
            # +base_type+ into the given intermediate type. Orogen will then use
            # that intermediate representation to marshal the data.
            #
            # Orogen has a specific support for data held by smart pointers. See #smart_ptr.
            #
            # The following options are available:
            # 
            # +:includes+ is an optional set of headers needed to define
            # +base_type+. For instance:
            #
            #   opaque_type "Eigen::Vector3f", "imu::Vector3f", :includes => ["/usr/include/eigen2", "/opt/boost/include"]
            #
            # If there is only one include, the array can be omitted
            #
            #   opaque_type "Eigen::Vector3f", "imu::Vector3f", :includes => "/usr/include/eigen2"
            #
            # +:needs_copy+ is a flag telling how the opaque type should
            # be converted. If true (the default), orogen will generate
            # convertion methods whose signature are:
            #
            #   void project_name::to_intermediate(intermediate_type& intermediate, base_type const& sample)
            #   void project_name::from_intermediate(base_type& sample, intermediate_type const& intermediate)
            #
            # In the first examples above, this would be (assuming an "imu" orogen project)
            #
            #   void imu::to_intermediate(imu::Vector3f& intermediate, Eigen::Vector3f const& sample)
            #   void imu::from_intermediate(Eigen::Vector3f& sample, imu::Vector3f& intermediate)
            #
            # Note that in from_intermediate, the +intermediate+ argument in
            # non-const. It is actually allows to modify it, as for instance to
            # get better performance (example: use +vector<>.swap()+ instead of
            # doing a big copy).
            #
            # If +:needs_copy+ is false, then we assume that a copy is not
            # needed. In that case, the to_intermediate convertion method will
            # return the intermediate type directly. The signature will
            # therefore be changed to
            #
            #   intermediate_type& project_name::to_intermediate(base_type const& sample)
            #   bool project_name::from_intermediate(base_type& sample, intermediate_type* intermediate)
            #
            # Note that in from_intermediate the +intermediate+ argument is now
            # given as a non-const pointer.  The convertion function can choose
            # to take ownership on that value, in which case it has to return
            # true. If the function returns false, then the sample is deleted after
            # the method call.
            def opaque_type(base_type, intermediate_type, options = {}, &convert_code_generator)
                options = validate_options options,
                    :includes => nil,
                    :needs_copy => true

                base_type = base_type.to_str
                if intermediate_type.kind_of?(Class) && intermediate_type < Typelib::Type
                    intermediate_type = intermediate_type.name
                end

                typedef = "<typelib><opaque name=\"#{base_type.gsub('<', '&lt;').gsub('>', '&gt;')}\" size=\"#{0}\" /></typelib>"
                opaque_def = Typelib::Registry.from_xml(typedef)
                opaque_registry.merge opaque_def
                registry.merge opaque_def
                component.registry.merge opaque_def

                opaque_type = find_type(base_type)
                orogen_def = OpaqueDefinition.new(opaque_type,
                                                 intermediate_type, options, convert_code_generator) 
                @opaques << orogen_def
            end

            # True if some opaques require to generate templates
            def has_opaques_with_templates?
                opaques.any? { |op| op.generate_templates? }
            end

            # call-seq:
            #   load(file)
            #
            # Load the types defined in the specified file.
            #
            # +file+ may contain pure-C code with the following C++ additions:
            # * namespaces
            # * <tt>std::vector< <i>type</i> ></tt>. You *have* to specify
            #   <tt>std::vector</tt> (and not simply +vector+, as the <tt>using
            #   namespace</tt> directive is not supported by orogen.
            # 
            # Moreover, the orogen tool defines the <tt>__orogen</tt>
            # preprocessor symbol when it loads the file. It is therefore
            # possible to define constructors, destructors, operators and (more
            # generically) methods by enclosing them in a block like
            #   #ifndef __orogen
            #   ... C++ code not supported by orogen ...
            #   #endif
            # 
            # <b>The use of virtual methods and inheritance is completely
            # forbidden</b>. The types need to remain "value types" without
            # inheritance. For those who want to know, this is needed so that
            # orogen is able to compute the memory layout of the types (i.e.
            # the exact offsets for all the fields in the structures).
	    def load(file, preload = false)
                # Find where +file+ is
                include_dirs = []
                include_dirs << component.base_dir if component.base_dir
                include_dirs.concat(component.used_libraries.map { |pkg| pkg.include_dirs }.flatten)
                include_dirs.concat(component.used_task_libraries.map { |component| component.pkg.include_dirs }.flatten)

                if File.exists?(file)
                    file = File.expand_path(file)
                else
                    dir = include_dirs.find { |dir| File.exists?(File.join(dir, file)) }
                    if !dir
                        raise ArgumentError, "cannot find #{file} in #{include_dirs.join(":")}"
                    end
                    file = File.join(dir, file)
                end

                file_registry = Typelib::Registry.new
                file_registry.merge opaque_registry

                options = { :define => '__orogen', :opaques_ignore => true }
                options[:include] = include_dirs.dup

                begin
                    file_registry.import(file, 'c', options)
                    registry.merge(file_registry)
                    preloaded_registry.merge(file_registry) if preload
                    component.registry.merge(file_registry)
                rescue RuntimeError => e
                    raise ArgumentError, "cannot load #{file}: #{e.message}", e.backtrace
                end

		loads << file
	    end

            # Packages defined in this component on which the toolkit should
            # depend. See #internal_dependency.
            attr_reader :internal_dependencies

            # The second one allows to specify a dependency of the toolkit on a
            # library defined in the same CMake package. The toolkit's .pc file
            # will therefore depend on the specified pkg-config package (among
            # other things). Do that when you define a library of types and
            # want people to be able to use it even though they don't have
            # orogen.
            #
            # In other words, it allows to build packages were:
            #  * a normal C/C++ library is defined/built/install
            #  * an orogen toolkit is defined, which wraps the types defined by this library
            def internal_dependency(name, version = nil)
                @internal_dependencies << [name, version]
            end

	    def import(other_toolkit)
		raise NotImplementedError
	    end

            # True if the CORBA-specific part of the toolkit should be enabled.
            # By default, it follows the setting in the component. You can a
            # per-toolkit specific setting by using #enable_corba and
            # #disable_corba
	    def corba_enabled?; @corba_enabled.nil? ? component.corba_enabled? : @corba_enabled end
            def enable_corba;  @corba_enabled = true end
	    def disable_corba; @corba_enabled = false end

	    def to_code(generated_types, registry)
		toolkit = self

                # Save all the types that this specific toolkit handles
                Generation.save_automatic "toolkit", "#{component.name}.typelist",
                    generated_types.
                        map { |type| type.name }.
                        join("\n")

                generate_typedefs(generated_types, registry)

		type_header = Generation.render_template('toolkit/types.hpp', binding)
		if corba_enabled?
		    corba  = Generation.render_template 'toolkit/corba.hpp', binding
		    idl    = Orocos::Generation.render_template "toolkit/corba.idl", binding
		end
		header = Orocos::Generation.render_template "toolkit/header.hpp", binding
		namespace = '/'
		source = Orocos::Generation.render_template "toolkit/toolkit.cpp", binding

		return type_header, header, source, corba, idl
	    end

            def opaque_specification(type_def)
                type = find_type(type_def)
                raise "#{type} is unknown" unless type
                raise "#{type} is not opaque" unless type.opaque?

                opaques.find { |opaque_def| opaque_def.type == type }
            end

            def intermediate_type?(type)
                opaques.find { |spec| find_type(spec.intermediate) == find_type(type.name) }
            end

            def m_type?(type)
                typename = type.name
                return false if typename !~ /_m$/
                begin
                    registry.get($`).contains_opaques?
                rescue Typelib::NotFound
                end
            end

            # Builds a map telling where are the opaque fields used in this
            # type. The returned value is an array of the form
            #  [
            #     [opaque_type, intermediate_type, path]
            #     ...
            #  ]
            # where the first two objects are subclasses of Typelib::Type and
            # the third is the "path" of the field in the structure (of the
            # form "a.b.c.d").
            def build_opaque_map(type)
                unless type < Typelib::CompoundType
                    raise "build_opaque_map is only valid for compound types"
                end

                result = []
                type.each_field do |field_name, field_type|
                    if field_type.opaque?
                        spec = opaque_specification(field_type)
                        result << [field_type, find_type(spec.intermediate), field_name]
                    elsif field_type < Typelib::CompoundType
                        inner = build_opaque_map(field_type)
                        if !inner.empty?
                            result.concat(inner.map { |a, b, path| [a, b, "#{field_name}.#{path}"] })
                        end
                    end
                end
                result
            end

            # This method performs sanity checks on the use of opaque types.
            # The only limitation is that an opaque opaque_t can only be
            # used at toplevel and in a struct. The following is forbidden:
            #
            #   struct S0
            #   {
            #       opaque_t field;
            #   };
            #
            #   struct S1
            #   {
            #       S0 field;
            #   };
            def validate_opaque_types
                registry.each_type do |type|
                    if type < Typelib::CompoundType
                        type.each_field do |_, field_type|
                            if !field_type.opaque? && field_type.contains_opaques?
                                raise NotImplementedError, "opaques types can only be used at toplevel and at one-level indirection"
                            end
                        end
                    end
                end
            end

            def handle_opaques_generation(generated_types, registry)
		toolkit = self

                validate_opaque_types

                # Make sure all opaque intermediate types are existing or can be
                # instanciated
                opaques.each do |opaque_def|
                    find_type(opaque_def.intermediate)
                end

                # Generate some type definitions for the pocosim marshalling. In
                # practice, we generate C code that we merge back into the
                # repository
                generate_all_marshalling_types = false
                catch(:nothing_to_define) do
                    Tempfile.open('orogen') do |io|
                        marshalling_code = Generation.render_template 'toolkit/marshalling_types.hpp', binding
                        io << marshalling_code
                        io.flush

                        registry.import(io.path, 'c')
                    end
                end
            end

            # This generates typedefs for container types. These
            # typedefs are needed because IDL and the CORBA C++ mapping do not
            # allow to reference sequence types directly (you have to typedef
            # them first, or use them in a compound)
            def generate_typedefs(generated_types, registry)
		generated_types.each do |type|
                    next if component.imported_type?(type.name)
		    if type < Typelib::ContainerType
                        registry.alias type.namespace + type.basename.gsub(/[^\w]/, '_'), type.name
		    end
		end
            end

	    def issue_warnings(generated_types, registry)
		generated_types.each do |type|
		    if type.contains_int64?
			Orocos::Generation.warn "you will not be able to marshal #{type.name} as XML, it contains 64bit integers"
		    end
		end
	    end

	    def generate
		toolkit = self

		# Remove all unneeded types from imported toolkits
		registry = self.registry.
		    minimal(preloaded_registry).
		    minimal(component.rtt_registry)
		registry = component.used_toolkits.
		    inject(registry) { |reg, tk| reg.minimal(tk.registry) }

		generated_types = []
		registry.each_type do |type|
                    next if component.imported_type?(type.name)
		    if !type.inlines_code?
			generated_types << type
		    end
		end

		issue_warnings(generated_types, registry)
                handle_opaques_generation(generated_types, registry)

                opaque_types = opaques.map { |opdef| opdef.type }
                opaque_intermediates = opaques.map do |opdef|
                    component.find_type(opdef.intermediate)
                end
                generate_typedefs(generated_types, registry)


		# if toolkit.corba_enabled?
		#     Generation.save_automatic("toolkit", "#{component.name}ToolkitCorba.hpp", corba)
		#     Generation.save_automatic("toolkit", "#{component.name}Toolkit.idl", idl)
		# end
		# Generation.save_automatic("toolkit", "#{component.name}Toolkit.hpp", hpp)
		# Generation.save_automatic("toolkit", "#{component.name}Toolkit.cpp", cpp)

                # Add opaque-related information in the TLB file
                plain_registry = registry.to_xml
                doc = Nokogiri::XML(plain_registry)
                doc.xpath('//opaque').each do |opaque_entry|
                    spec = opaque_specification(opaque_entry['name'])

                    opaque_entry['marshal_as'] = spec.intermediate
                    opaque_entry['includes']   = spec.includes.join(':')
                end
                Generation.save_automatic "toolkit", "#{component.name}.tlb", doc.to_xml

                # Save all the types that this specific toolkit handles
                Generation.save_automatic "toolkit", "#{component.name}.typelist",
                    generated_types.
                        map { |type| type.name }.
                        join("\n")

                # The first array is the set of types for which convertion
                # functions are generated. The second is the set of types that
                # are actually registered into the RTT type system
                #
                # The two arrays are sorted so that we don't have to recompile
                # unncessary (the original sets are hashes, and therefore don't
                # have a stable order).
                converted_types = (generated_types + opaque_types + opaque_intermediates).
                    find_all { |type| !toolkit.m_type?(type) }.
                    sort_by { |type| type.name }
                registered_types = (generated_types + opaque_types + opaque_intermediates).
                    find_all { |type| !toolkit.m_type?(type) && !(type <= Typelib::ArrayType) }.
                    sort_by { |type| type.name }

                # Generate the C++ and IDL files
                tk_hpp = Generation.render_template "toolkit/Toolkit.hpp", binding
		Generation.save_automatic("toolkit", "#{component.name}Toolkit.hpp", tk_hpp)
                tk_cpp = Generation.render_template "toolkit/Toolkit.cpp", binding
		Generation.save_automatic("toolkit", "#{component.name}Toolkit.cpp", tk_cpp)
                tk_impl_hpp = Generation.render_template "toolkit/ToolkitImpl.hpp", binding
		Generation.save_automatic("toolkit", "#{component.name}ToolkitImpl.hpp", tk_impl_hpp)
		type_header = Generation.render_template('toolkit/ToolkitTypes.hpp', binding)
		Generation.save_automatic("toolkit", "#{component.name}ToolkitTypes.hpp", type_header)
		pkg_config = Generation.render_template 'toolkit/toolkit.pc', binding
		Generation.save_automatic("toolkit", "#{component.name}-toolkit.pc.in", pkg_config)

                if corba_enabled?
                    corba_hpp = Generation.render_template "toolkit/ToolkitCorba.hpp", binding
                    Generation.save_automatic("toolkit", "#{component.name}ToolkitCorba.hpp", corba_hpp)
                    corba_impl_hpp = Generation.render_template "toolkit/ToolkitCorbaImpl.hpp", binding
                    Generation.save_automatic("toolkit", "#{component.name}ToolkitCorbaImpl.hpp", corba_impl_hpp)
                    corba_cpp = Generation.render_template "toolkit/ToolkitCorba.cpp", binding
                    Generation.save_automatic("toolkit", "#{component.name}ToolkitCorba.cpp", corba_cpp)
		    idl    = Orocos::Generation.render_template "toolkit/Toolkit.idl", binding
                    Generation.save_automatic "toolkit", "#{component.name}Toolkit.idl", idl
                    pkg_config = Generation.render_template 'toolkit/transport-corba.pc', binding
                    Generation.save_automatic("toolkit", "#{component.name}-transport-corba.pc.in", pkg_config)
		end

                # Generate the pkg-config file

                # Generate the user part for opaque types
                if !opaques.empty?
                    intermediates = Generation.render_template 'toolkit/intermediates.hpp', binding
                    Generation.save_automatic("toolkit", "#{component.name}ToolkitIntermediates.hpp", intermediates)
                    if has_opaques_with_templates?
                        user_hh = Generation.render_template 'toolkit/user.hpp', binding
                        user_cc = Generation.render_template 'toolkit/user.cpp', binding
                        Generation.save_user 'toolkit', "#{component.name}ToolkitUser.hpp", user_hh
                        Generation.save_user 'toolkit', "#{component.name}ToolkitUser.cpp", user_cc
                    end
                end

                # Finished, create the timestamp file
                FileUtils.touch File.join(Generation::AUTOMATIC_AREA_NAME, 'toolkit', 'stamp')
	    end
	end
    end
end

