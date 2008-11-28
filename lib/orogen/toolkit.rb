require 'typelib'
require 'tempfile'
require 'find'
require 'orogen/base'
require 'utilrb/kernel/options'
require 'nokogiri'

module Typelib
    class Type
        def self.cxx_name(fullname = true)
            full_name('::', fullname).gsub("<::", "<")
        end
        def self.cxx_basename
            basename('::').gsub("<::", "<")
        end
        def self.cxx_namespace
            namespace('::').gsub("<::", "<")
        end

        def self.corba_name
            cxx_name
        end
        def self.method_name(fullname = true)
            base = if fullname then full_name('_', true)
                   else basename('_')
                   end
            base.gsub(/[<>\[\]]/, '_')
        end

        @@index_var_stack = Array.new
        def self.index_var_stack; @@index_var_stack end
        def self.allocate_index
            index_var_stack.push "i#{index_var_stack.size}"
            yield(index_var_stack.last)
        ensure
            index_var_stack.pop
        end

	def self.from_orocos_decomposition(toolkit, result, path, indent = "    ")
            if opaque?
                property_name = path[1..-1]
                result << indent << "{ PropertyBag const& inner_bag = bag.getProperty<PropertyBag>(\"#{property_name}\")->value();\n"
                result << indent << "   #{cxx_name}TypeInfo::doCompose(inner_bag, out#{path});\n"
                result << indent << "}"
            else
                orocos_type = registry.orocos_equivalent(self).cxx_name
                property_name = path[1..-1]
                result << indent << "out#{path} = bag.getProperty<#{orocos_type}>(\"#{property_name}\")->get();"
            end
	end

	def self.to_orocos_decomposition(toolkit, result, path, indent = "    ")
            if opaque?
                result << indent << "{ PropertyBag inner_bag(\"#{full_name}\");\n"
                result << indent << "   #{cxx_namespace}#{method_name(false)}TypeInfo::doDecompose(value#{path}, inner_bag);\n"
                property_name = path[1..-1]
                result << indent << "   Property<PropertyBag>* temp_property = new Property<PropertyBag>(\"#{property_name}\", \"\", inner_bag);\n"
                result << indent << "   target_bag.add(temp_property);\n"
                result << indent << "}"
            else
                orocos_type = registry.orocos_equivalent(self).cxx_name
                property_name = path[1..-1]
                result << indent << "target_bag.add( new Property<#{orocos_type}>(\"#{property_name}\", \"\", value#{path}) );"
            end
	end

	def self.code_to_corba(toolkit, result, path = "", indent = "    ")
            if opaque?
                result << indent << "result#{path} = *AnyConversion< #{cxx_name} >::createAny(value#{path});"
            else
                result << indent << "result#{path} = value#{path};"
            end
	end
	def self.code_from_corba(toolkit, result, path = "", indent = "    ")
            if opaque?
                result << indent << "AnyConversion< #{cxx_name} >::updateFromAny(value#{path}, result#{path});"
            else
                result << indent << "result#{path} = value#{path};"
            end
	end
        def self.to_ostream(toolkit, result, path, indent)
            if opaque?
                result << indent << "io << data#{path};"
            else
                orocos_type = registry.orocos_equivalent(self).cxx_name
                property_name = path[1..-1]
                result << indent << "io << static_cast<#{orocos_type}>(data#{path});"
            end
        end
    end

    class ContainerType
        def self.collection_iteration(varname, result, path, indent)
            collection_name, element_type = container_kind, deference.name
            element_type = registry.build(element_type)

            result << "#{indent}for(#{cxx_name}::const_iterator it = #{varname}#{path}.begin(); it != #{varname}#{path}.end(); ++it)\n"
            result << "#{indent}{\n"
            yield(element_type)
            result << "#{indent}}\n"
        end

	def self.to_orocos_decomposition(toolkit, result, path, indent = "    ")
            collection_iteration(:value, result, path, indent) do |element_type|
                result << "#{indent}  #{element_type.cxx_name} const& value = *it;\n"
                element_type.to_orocos_decomposition(toolkit, result, "", indent + "  ") << "\n"
            end
	end

	def self.code_to_corba(toolkit, result, path = "", indent = "    ")
            result << "#{indent}result#{path}.length(value#{path}.size());\n"
            allocate_index do |index_var|
                result << "#{indent}int #{index_var} = 0;\n"
                collection_iteration(:value, result, path, indent) do |element_type|
                    result << "#{indent}  #{element_type.cxx_name} const& value = *it;\n"
                    result << "#{indent}  CorbaType& real_result = result;\n"
                    result << "#{indent}  #{element_type.corba_name} result;\n"
                    element_type.code_to_corba(toolkit, result, "", indent + "  ") << "\n";
                    result << "#{indent}  real_result#{path}[#{index_var}] = result;\n"
                    result << "#{indent}  ++#{index_var};\n"
                end
            end
	end
	def self.code_from_corba(toolkit, result, path = "", indent = "    ")
            result << "#{indent}result#{path}.resize(value#{path}.length());\n"
            allocate_index do |index_var|
                result << "#{indent}int #{index_var} = 0;\n"
                collection_iteration(:result, result, path, indent) do |element_type|
                    result << "#{indent}  CorbaType const& real_value = value;\n"
                    result << "#{indent}  #{element_type.corba_name} const& value = real_value#{path}[#{index_var}];\n"
                    result << "#{indent}  #{element_type.cxx_name} result;\n"
                    element_type.code_from_corba(toolkit, result, "", indent + "  ") << "\n";
                    result << "#{indent}  ++#{index_var};\n"
                end
            end
	end
	def self.from_orocos_decomposition(toolkit, result, path, indent = "    ")
	end
        def self.to_ostream(toolkit, result, path, indent)
            collection_iteration(:data, result, path, indent) do |element_type|
                result << "#{indent}  #{element_type.cxx_name} const& data = *it;\n"
                element_type.to_ostream(toolkit, result, "", indent + "  ") << "\n"
                result << "#{indent}  io << \", \";\n"
            end
        end
    end

    class EnumType
	def self.to_orocos_decomposition(toolkit, result, path, indent = "    ")
            property_name = path[1..-1]
            result << indent << "{ std::string enum_name;\n"
            result << indent << "  switch(value#{path}) {\n"
            keys.each do |name, _|
                result << indent << "    case #{name}:\n"
                result << indent << "      enum_name = \"#{name}\";\n"
                result << indent << "      break;\n"
            end
	    result << indent << "    default:\n"
	    result << indent << "      enum_name = boost::lexical_cast<std::string>(value#{path});\n"
            result << indent << "      break;\n"
            result << indent << "  }\n"
            result << indent << "  target_bag.add( new Property<std::string>(\"#{property_name}\", \"\", enum_name) );\n"
            result << indent << "}"
	end
	def self.from_orocos_decomposition(toolkit, result, path, indent = "    ")
            property_name = path[1..-1]
            result << indent << "{ std::string enum_name;\n"
            result << indent << "  enum_name = bag.getProperty<std::string>( \"#{property_name}\" )->get();\n"
            keys.each do |name, _|
		result << indent << "  if(enum_name == \"#{name}\")\n"
		result << indent << "     out#{path} = #{name};\n"
            end
            result << indent << "}"
	end


        def self.to_ostream(toolkit, result, path, indent)
            property_name = path[1..-1]
            result << indent << "switch(data#{path}) {\n"
            keys.each do |name, _|
                result << indent << "  case #{name}:\n"
                result << indent << "    io << \"#{name}\";\n"
                result << indent << "    break;\n"
            end
	    result << indent << "    default:\n"
	    result << indent << "      io << data#{path};\n"
            result << indent << "      break;\n"
            result << indent << "}\n"
        end
	def self.code_to_corba(toolkit, result, path = "", indent = "    ")
            namespace = self.namespace('::')
            result << indent << "switch(value#{path}) {\n"
            keys.each do |name, _|
                result << indent << "  case #{namespace}#{name}:\n"
                result << indent << "    result#{path} = #{namespace}Corba::#{name};\n"
                result << indent << "    break;\n"
            end
            result << indent << "}\n"

	    result
	end
	def self.code_from_corba(toolkit, result, path = "", indent = "    ")
            namespace = self.namespace('::')
            result << indent << "switch(value#{path}) {\n"
            keys.each do |name, _|
                result << indent << "  case #{namespace}Corba::#{name}:\n"
                result << indent << "    result#{path} = #{namespace}#{name};\n"
                result << indent << "    break;"
            end
            result << indent << "}\n"

	    result
	end
    end

    class CompoundType
        def self.corba_name
            "#{namespace('::')}Corba::#{basename.gsub('<::', '< ::')}"
        end
	def self.convertion_code_helper(method, toolkit, result, path, indent)
	    each_field do |name, type|
		type.send(method, toolkit, result, "#{path}.#{name}", indent) << "\n"
	    end
	end
	def self.to_orocos_decomposition(toolkit, result, path, indent = "    ")
	    convertion_code_helper(:to_orocos_decomposition, toolkit, result, path, indent)
	    result
	end
	def self.code_to_corba(toolkit, result, path = "", indent = "    ")
	    convertion_code_helper(:code_to_corba, toolkit, result, path, indent)
	    result
	end
	def self.code_from_corba(toolkit, result, path = "", indent = "    ")
	    convertion_code_helper(:code_from_corba, toolkit, result, path, indent)
	    result
	end

	def self.from_orocos_decomposition(toolkit, result, path, indent = "    ")
	    convertion_code_helper(:from_orocos_decomposition, toolkit, result, path, indent)
	    result
	end

        def self.to_ostream(toolkit, result, path, indent)
            result << indent << "io << \"{ \";\n"

            first_field = true
	    each_field do |name, type|
                unless first_field
                    result << "#{indent}  io << \", \";\n";
                end

                first_field = false
		result << "#{indent}  io << \"#{name} = \";\n"
                type.to_ostream(toolkit, result, "#{path}.#{name}", indent + "  ") << "\n"
	    end
            result << indent << "io << \" }\";\n"
        end
    end
    class ArrayType
        def self.corba_name; raise NotImplementedError end

	def self.convertion_code_helper(method, toolkit, result, path, indent)
	    length.times do |i|
                deference.send(method, result, path + "[#{i}]", indent + "    ") << "\n"
            end
	    result
	end
	def self.to_orocos_decomposition(toolkit, result, path, indent = "    ")
	    convertion_code_helper(:to_orocos_decomposition, toolkit, result, path, indent)
	end
	def self.code_to_corba(toolkit, result, path = "", indent = "    ")
	    convertion_code_helper(:code_to_corba, toolkit, result, path, indent)
	end
	def self.code_from_corba(toolkit, result, path = "", indent = "    ")
	    convertion_code_helper(:code_from_corba, toolkit, result, path, indent)
	end

	def self.from_orocos_decomposition(toolkit, result, path, indent = "    ")
	    convertion_code_helper(:from_orocos_decomposition, toolkit, result, path, indent)
	end

        def self.to_ostream(toolkit, result, path, indent)
            result << indent << "io << \"[\\n\";\n"
            allocate_index do |index_var|
                result << indent << "for (int #{index_var} = 0; #{index_var} < #{length}; ++#{index_var}) {\n" 
                    deference.to_ostream(toolkit, result, "#{path}[#{index_var}]", indent + "  ")
                    result << "#{indent}  if (#{index_var} != #{length - 1}) io << \", \";\n"
                result << indent << "}\n" 
            end
            result << indent << "io << \" ]\";\n"
        end
    end
    class Registry
	OROCOS_KNOWN_TYPES = ['int', 'unsigned int', 'float', 'double']
	OROCOS_KNOWN_CONVERTIONS = {
		'char' => 'int',
		'unsigned char'  => 'unsigned int',
		'unsigned short' => 'unsigned int',
		'short' => 'int' }

	attr_reader :orocos_type_equivalence
	def build_orocos_type_equivalence
	    @orocos_type_equivalence = Hash.new

	    orocos_known_types = OROCOS_KNOWN_TYPES.map { |name| get(name) }
	    orocos_known_convertions = OROCOS_KNOWN_CONVERTIONS.map { |from, to| [get(from), get(to)] }
	    each_type do |type|
		next if type < CompoundType || type < ArrayType

		if eqv = orocos_known_types.find { |known_t| known_t == type }
		    orocos_type_equivalence[type] = eqv
		elsif eqv = orocos_known_convertions.find { |from_t, to_t| from_t == type }
		    orocos_type_equivalence[type] = eqv[1]
		end
	    end
	end

	def orocos_equivalent(user_type)
	    if !orocos_type_equivalence
		build_orocos_type_equivalence
	    end

	    if type = orocos_type_equivalence[user_type]
		type
            elsif user_type < EnumType
                get("/std/string")
            else
		raise TypeError, "#{user_type.name} does not have an equivalent in the Orocos RTT toolkit"
	    end
	end
    end
end

module Orocos
    module Generation
        OpaqueDefinition = Struct.new :type, :intermediate, :includes

	class Toolkit
	    attr_reader :component
	    attr_reader :imports, :loads
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

            # True if we are generating for Linux
            def linux?;     component.linux? end
            # True if we are generating for Xenomai
            def xenomai?;   component.xenomai? end

	    def initialize(component, &block)
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

		instance_eval(&block) if block_given?
	    end

            # Loads the types defined in +file+, with the same constraints as
            # for #load, but will not generate toolkit code for it. This is
            # needed to load system files for which there is a "natural"
            # support in Orocos.
            def preload(file)
                load(file, true)
            end

            # Declare that the user will provide a method which converts
            # +base_type+ into the given intermediate type. Orogen will then use
            # that intermediate representation to marshal the data.
            #
            # +base_type+ is defined as an opaque type in the component's
            # registry
            #
            # +includes+ is an optional set of headers needed to define
            # +base_type+
            def opaque_type(base_type, intermediate_type, options = {})
                options = validate_options options,
                    :includes => nil

                if base_type.respond_to?(:to_str)
                    typedef = "<typelib><opaque name=\"#{base_type.gsub('<', '&lt;').gsub('>', '&gt;')}\" size=\"#{0}\" /></typelib>"

                    opaque_def = Typelib::Registry.from_xml(typedef)
                    opaque_registry.merge opaque_def
                    registry.merge opaque_def
                    component.registry.merge opaque_def
                end

                @opaques << OpaqueDefinition.new(component.find_type(base_type),
                                                 intermediate_type,
                                                 options[:includes])
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
                cpp_source = Tempfile.new("preload", Dir.pwd)
                cpp_source.puts "#include <#{file}>"
                cpp_source.flush

                file_registry = Typelib::Registry.new
                file_registry.merge opaque_registry

                options = { :define => '__orogen', :opaques_ignore => true }
                options[:include] = []
                options[:include] << component.base_dir if component.base_dir
                component.used_libraries.each do |pkg|
                    options[:include] << pkg.includedir
                end
                component.used_task_libraries.each do |pkg|
                    options[:include] << pkg.includedir
                end

		file_registry.import(cpp_source.path, 'c', options)

                registry.merge(file_registry)
                preloaded_registry.merge(file_registry) if preload
		component.registry.merge(file_registry)

		loads << file
            ensure
                cpp_source.close if cpp_source
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

	    def to_code
		toolkit = self

                registry = self.registry.minimal(preloaded_registry)
		type_header = Generation.render_template('toolkit/types.hpp', binding)

		generated_types = []
		registry.each_type do |type|
		    if type < Typelib::CompoundType && !component.used_toolkits.any? { |_, r| r.get(type.name) }
			generated_types << type
		    end
		end

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
                type = component.find_type(type_def)
                raise "#{type} is unknown" unless type
                raise "#{type} is not opaque" unless type.opaque?

                opaques.find { |opaque_def| opaque_def.type == type }
            end

            def contains_opaques?(type)
                type = component.find_type(type)
                return true  if type.opaque?
                return false unless type < Typelib::CompoundType

                type.each_field do |field_name, field_type|
                    return true if contains_opaques?(field_type)
                end
                false
            end

            def intermediate_type?(type)
                opaques.find { |opaque_def| component.find_type(opaque_def.intermediate) == type }
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
                        result << [field_type, component.find_type(spec.intermediate), field_name]
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
                            if !field_type.opaque? && contains_opaques?(field_type)
                                raise NotImplementedError, "opaques types can only be used at toplevel and at one-level indirection"
                            end
                        end
                    end
                end
            end

            def handle_opaques_generation
		toolkit = self

                validate_opaque_types

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

	    def generate
		toolkit = self
                handle_opaques_generation

		types, hpp, cpp, corba, idl = to_code
		if toolkit.corba_enabled?
		    Generation.save_automatic("toolkit", "#{component.name}ToolkitCorba.hpp", corba)
		    Generation.save_automatic("toolkit", "#{component.name}Toolkit.idl", idl)
		end
		Generation.save_automatic("toolkit", "#{component.name}ToolkitTypes.hpp", types)
		Generation.save_automatic("toolkit", "#{component.name}Toolkit.hpp", hpp)
		Generation.save_automatic("toolkit", "#{component.name}Toolkit.cpp", cpp)

                # Add opaque-related information in the TLB file
                plain_registry = registry.to_xml
                doc = Nokogiri::XML(plain_registry)
                doc.xpath('//opaque').each do |opaque_entry|
                    spec = opaque_specification(opaque_entry['name'])

                    opaque_entry['marshal_as'] = spec.intermediate
                    opaque_entry['includes']   = spec.includes.join(':')
                end
                Generation.save_automatic "toolkit", "#{component.name}.tlb", doc.to_xml

		pkg_config = Generation.render_template 'toolkit/toolkit.pc', binding
		Generation.save_automatic("toolkit", "#{component.name}-toolkit.pc.in", pkg_config)

                if !opaques.empty?
                    user_hh = Generation.render_template 'toolkit/user.hpp', binding
                    user_cc = Generation.render_template 'toolkit/user.cpp', binding
                    Generation.save_user 'toolkit', "#{component.name}ToolkitUser.hpp", user_hh
                    Generation.save_user 'toolkit', "#{component.name}ToolkitUser.cpp", user_cc
                end
	    end
	end
    end
end

