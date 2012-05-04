# -*- coding: utf-8 -*-
require 'typelib'
require 'tempfile'
require 'find'
require 'orogen/base'
require 'utilrb/kernel/options'
require 'nokogiri'

module Typelib
    class Type
        def self.normalize_typename(name)
            "/" + Typelib.split_typename(name).map do |part|
                normalize_typename_part(part)
            end.join("/")
        end

        def self.normalize_typename_part(name)
            # Remove all trailing array modifiers first
            if name =~ /(.*)(\[\d+\]+)$/
                name, array_modifiers = $1, $2
            end

            name, template_arguments = Typelib::GCCXMLLoader.parse_template(name)
            template_arguments.map! do |arg|
                arg =
                    if arg !~ /^\d+$/ && arg[0, 1] != "/"
                        "/#{arg}"
                    else arg
                    end
            end

            if !template_arguments.empty?
                "#{name}<#{template_arguments.join(",")}>#{array_modifiers}"
            else
                "#{name}#{array_modifiers}"
            end
        end

        def self.normalize_cxxname(name)
            if name =~ /::/
                raise Orocos::Generation::InternalError, "normalize_cxxname called with a C++ type name (#{name})"
            end

            if name =~ /(.*)((?:\[\d+\])+)$/
                name = $1
                suffix = $2
            else
                suffix = ''
            end

            converted = Typelib.split_typename(name).map do |p|
                normalize_cxxname_part(p)
            end
            if converted.size == 1
                "#{converted.first}#{suffix}"
            else
                "::" + converted.join("::") + suffix
            end
        end

        def self.normalize_cxxname_part(name)
            name, template_arguments = Typelib::GCCXMLLoader.parse_template(name)

            name = name.gsub('/', '::')
            if name =~ /^::(.*)/
                name = $1
            end

            if !template_arguments.empty?
                template_arguments.map! do |arg|
                    if arg !~ /^\d+$/
                        normalize_cxxname(arg)
                    else arg
                    end
                end

                "#{name}< #{template_arguments.join(", ")} >"
            else name
            end
        end

        def self.cxx_name
            normalize_cxxname(name)
        end
        def self.cxx_basename
            normalize_cxxname(basename)
        end
        def self.cxx_namespace
            namespace('::')
        end
        def self.name_as_word
            cxx_name.gsub(/[^\w+]/, '_')
        end

        def self.arg_type; "#{cxx_name} const&" end
        def self.ref_type; "#{cxx_name}&" end

        def self.method_name(fullname = true)
            base = if fullname then full_name('_', true)
                   else basename('_')
                   end
            base.gsub(/[<>\[\], ]/, '_')
        end

	def self.contains_int64?
            dependencies.any? { |t| t.contains_int64? }
        end
        def self.contains_opaques?
            if @contains_opaques.nil?
                @contains_opaques = contains?(Typelib::OpaqueType)
            end
            @contains_opaques
        end

        @@index_var_stack = Array.new
        def self.index_var_stack; @@index_var_stack end
        def self.allocate_index
            index_var_stack.push "i#{index_var_stack.size}"
            yield(index_var_stack.last)
        ensure
            index_var_stack.pop
        end

        def self.inlines_code?; false end

        def self.code_assign(result, indent, dest, src)
            result << "#{indent}#{dest} = #{src};\n"
        end
    end

    class NumericType
	def self.contains_int64?
            integer? && size == 8
        end

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

        def self.inlines_code?; superclass.eql?(NumericType) end
    end

    class ContainerType
        def self.to_m_type(target_basename, typekit)
            target_cxxname = typekit.intermediate_cxxname_for(deference)
            "struct __gccxml_workaround_#{method_name(true)} {\n  #{container_cxx_kind}< #{target_cxxname[0]}#{target_cxxname[1]} > instanciate;\n};\n"  +
            "typedef #{container_cxx_kind}< #{target_cxxname[0]}#{target_cxxname[1]} > orogen_typekits_mtype_#{method_name(true)};"
        end

        def self.container_cxx_kind
            normalize_cxxname(container_kind)
        end


        def self.cxx_name
            if name =~ /</
                normalize_cxxname(container_kind) + "< " + deference.cxx_name + " >"
            else
                normalize_cxxname(container_kind)
            end
        end

        def self.code_copy(typekit, result, indent, dest, src, method, src_type, dest_type)
            collection_name, element_type = container_kind, deference.name
            element_type = registry.build(element_type)

	    result << "#{indent}#{dest}.clear();\n"
            if (container_kind == "vector")
                result << "#{indent}#{dest}.reserve(#{src}.size());\n"
            end

            result << <<-EOT
#{indent}for(#{src_type.cxx_name}::const_iterator it = #{src}.begin(); it != #{src}.end(); ++it)
#{indent}{
            EOT

            if yield(element_type)
                result << "#{indent}    #{dest}.push_back(*it);\n"
            else
                result << "#{indent}    #{dest}.push_back(#{dest_type.deference.cxx_name}());\n"
                result << "#{indent}    #{method}(*#{dest}.rbegin(), *it);\n";
            end

            result << "#{indent}}\n";
        end

        def self.to_intermediate(typekit, result, indent)
            code_copy(typekit, result, indent, "intermediate", "value", "toIntermediate", self, typekit.intermediate_type_for(self)) do |type|
                !type.contains_opaques?
            end
        end
        def self.from_intermediate(typekit, result, indent)
            code_copy(typekit, result, indent, "value", "intermediate", "fromIntermediate", typekit.intermediate_type_for(self), self) do |type|
                !type.contains_opaques?
            end
        end
    end

    class EnumType
        def self.to_string(typekit, result, indent)
            seen_values = Set.new
            result << indent << "std::string enum_name;\n"
            result << indent << "switch(value) {\n"
            keys.each do |name, value|
                next if seen_values.include?(value)
                seen_values << value

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

    end

    class CompoundType
        def self.to_m_type(target_basename, typekit)
            result = <<-EOCODE
struct #{target_basename}
{
            EOCODE
            each_field do |field_name, field_type|
                field_cxxname =
                    if field_type.contains_opaques?
                        typekit.intermediate_cxxname_for(field_type.name)
                    elsif field_type <= ArrayType
                        [field_type.deference.cxx_name, "[#{field_type.length}]"]
                    else
                        [field_type.cxx_name]
                    end
                result << "   #{field_cxxname[0]} #{field_name} #{field_cxxname[1]};\n"
            end
            result << "};"
        end

        def self.code_copy(typekit, result, indent, dest, src, method, error_handling)
            each_field do |field_name, field_type|
                if string = yield(field_name, field_type)
                    if !string.respond_to?(:to_str)
                        result << "#{indent}#{dest}.#{field_name} = #{src}.#{field_name};\n"
                    else
                        result << string
                    end
                else
                    s_src  = "#{src}.#{field_name}"
                    s_dest = "#{dest}.#{field_name}"
                    if field_type < ArrayType
                        if src == "value"
                            s_src = "reinterpret_cast< #{field_type.deference.cxx_name} const*>(#{s_src}), #{field_type.length}"
                        else
                            s_dest = "reinterpret_cast< #{field_type.deference.cxx_name}*>(#{s_dest}), #{field_type.length}"
                        end
                    end

                    if error_handling
                        result << "#{indent}if (!#{method}(#{s_dest}, #{s_src})) return false;\n";
                    else
                        result << "#{indent}#{method}(#{s_dest}, #{s_src});\n"
                    end
                end
            end
	    result
        end

        def self.to_intermediate(typekit, result, indent)
            code_copy(typekit, result, indent, "intermediate", "value", "toIntermediate", false) do |field_name, field_type|
                if !field_type.contains_opaques?
                    field_type.code_assign([], indent + "    ", "intermediate.#{field_name}", "value.#{field_name}").join
                end
            end
        end
        def self.from_intermediate(typekit, result, indent)
            code_copy(typekit, result, indent, "value", "intermediate", "fromIntermediate", false) do |field_name, field_type|
                if !field_type.contains_opaques?
                    field_type.code_assign([], indent + "    ", "value.#{field_name}", "intermediate.#{field_name}").join
                end
            end
        end
    end

    class ArrayType
        def self.arg_type; "#{deference.cxx_name} const*" end
        def self.ref_type; "#{deference.cxx_name}*" end

        def self.to_m_type(target_basename, typekit)
            deference_name = typekit.intermediate_cxxname_for(deference)
            "typedef #{deference_name[0]} orogen_typekits_mtype_#{target_basename}#{deference_name[1]};"
        end

        def self.code_assign(result, indent, dest, src)
            element_type = deference
            allocate_index do |i|
                result << <<-EOT
#{indent}for(int #{i} = 0; #{i} < #{length}; ++#{i})
#{indent}{
                EOT
                deference.code_assign(result, "#{indent}  ", "#{dest}[#{i}]", "#{src}[#{i}]")
                result << "#{indent}}"
            end
        end

        def self.code_copy(typekit, result, indent, dest, src, method)
            element_type = registry.build(deference.name)

            allocate_index do |i|
                result << <<-EOT
#{indent}for(int #{i} = 0; #{i} < length; ++#{i})
#{indent}{
                EOT

                if string = yield(element_type, indent, "#{dest}[#{i}]", "#{src}[#{i}]")
                    if !string.respond_to?(:to_str)
                        result << "#{indent}  #{dest}[#{i}] = #{src}[#{i}];\n"
                    else
                        result << string
                    end
                else
                    s_src  = "#{src}[#{i}]"
                    s_dest = "#{dest}[#{i}]"
                    if element_type < ArrayType
                        if src == "value"
                            s_src = "reinterpret_cast< #{element_type.deference.cxx_name} const*>(#{s_src}), #{element_type.length}"
                        else
                            s_dest = "reinterpret_cast< #{element_type.deference.cxx_name}*>(#{s_dest}), #{element_type.length}"
                        end
                    end

                    result << "#{indent}    #{method}(#{s_dest}, #{s_src});\n";
                end

                result << "#{indent}}\n";
            end
	    result
        end

        def self.to_intermediate(typekit, result, indent)
            code_copy(typekit, result, indent, "intermediate", "value", "toIntermediate") do |type, indent, dest, src|
                if !type.contains_opaques?
                    type.code_assign([], indent, dest, src).join
                end
            end
        end

        def self.from_intermediate(typekit, result, indent)
            code_copy(typekit, result, indent, "value", "intermediate", "fromIntermediate") do |type, indent, dest, src|
                if !type.contains_opaques?
                    type.code_assign([], indent, dest, src).join
                end
            end
        end
    end

    class Registry
        # Returns true if +type+ is handled by the typekit that is included in
        # the RTT itself, and false otherwise.
        #
        # This is used in property bags and in the interface definition, as --
        # among the simple types -- only these can be used directly in
        # interfaces.
        def self.base_rtt_type?(type)
            if type.name == "/std/string"
                return true
            elsif !(type <= Typelib::NumericType)
                return false
            end

            if type.integer?
                type.name == "/bool" || type.size == 4
            else
                type.name == "/double"
            end
        end

        # Returns the RTT type that we should use to handle +type+.
        #
        # This is used in property bag marshalling/demarshalling to convert to
        # the corresponding property type.
        def base_rtt_type_for(type)
	    if Registry.base_rtt_type?(type)
		type
            elsif type < Typelib::NumericType
                if type.integer?
                    if type.size == 8
                        raise NotImplementedError, "there is no RTT type for 64bit integers, sorry"
                    elsif type.unsigned? then get("/unsigned int")
                    else get("/int")
                    end
                else
                    get("/double")
                end
            else
                raise ArgumentError, "no type equivalent for #{type.name} in Orocos"
            end
        end
    end
end

module Orocos
    module Generation
        def self.create_or_update_symlink(source, target)
            if File.exists?(target)
                if !File.symlink?(target)
                    raise ConfigError, "#{target} was expected to be a symbolic link, but is not"
                end
                pointed_to = File.readlink(target)
                return if pointed_to == target
            end
            FileUtils.mkdir_p(File.dirname(target))
            FileUtils.ln_sf(source, target)
        end

        # Common implementation of opaque-related methods. It gets included in
        # Project and Typekit
        module OpaqueHandling
            def opaque_specification(type_def)
                type = find_type(type_def)
                raise "#{type} is unknown" unless type
                raise "#{type} is not opaque" unless type.opaque?
                if result = opaques.find { |opaque_def| opaque_def.type == type }
		    result
		else
		    raise InternalError, "#{self}#opaque_specification called for type #{type.name}, but could not find the corresponding opaque specification"
		end
            end

            def find_opaque_for_intermediate(type)
                type = find_type(type.name)
                if m_type?(type)
                    # Yuk
                    begin
                        if @intermediate_to_opaque && (result = @intermediate_to_opaque[type.name])
                            result
                        else
                            find_type(type.name.gsub(/_m$/, ''))
                        end
                    rescue Typelib::NotFound
                        # This is a pretty expensive operation and is seldom
                        # needed, so avoid doing it unnecessarily
                        @intermediate_to_opaque ||= Hash.new
                        @indexed_intermediates ||= Set.new
                        registry.each do |t|
                            if !@indexed_intermediates.include?(t) && t.contains_opaques?
                                @indexed_intermediates << t
                                @intermediate_to_opaque[intermediate_type_name_for(t)] = t
                            end
                        end
                        @intermediate_to_opaque[type.name]
                    end
                else
                    opaques.find { |spec| find_type(spec.intermediate, true) == type }
                end
            end

            def intermediate_type_name_for(type_def, is_normalized = false)
                type = find_type(type_def, is_normalized)
                if type.opaque?
                    opaque_specification(type_def).intermediate
                elsif type.contains_opaques?
                    if type < Typelib::ArrayType
                        "#{intermediate_type_name_for(type.deference)}[#{type.length}]"
                    elsif type < Typelib::ContainerType
                        "#{type.container_kind}<#{intermediate_type_name_for(type.deference)}>"
                    else
                        path = Typelib.split_typename(type.name)
                        path.map! do |p|
                            p.gsub(/[<>\[\], \/]/, '_')
                        end
			"/" + path.join("/") + "_m"
                    end
                else type.name
                end
            end

            def intermediate_type_for(type_def)
                typename = intermediate_type_name_for(type_def)
                return find_type(typename, true)
            end

            def intermediate_type?(type)
                !!find_opaque_for_intermediate(type)
            end

            def m_type?(type)
                typename = type.name
                if type.name =~ /_m$/
                    return true
                end

                type.recursive_dependencies.any? { |t| t =~ /_m$/ }
            end
        end

        class << self
            attr_accessor :typekit_slice
            attr_accessor :typekit_slice_minimum
        end
        # The default value of 2 has been benchmarked with GCC 4.4.5. Big
        # transition from 1 to 2, but no big improvement afterwards.
        @typekit_slice = 2
        @typekit_slice_minimum = 1

        # Data structure that represents the definition for an opaque type
        #
        # See Typekit#opaque_type
        class OpaqueDefinition
            # The Typelib::Type subclass that represents this opaque
            attr_reader :type
            # The Typelib::Type subclass that represents the intermediate type
            # used to marshal this opaque
            attr_reader :intermediate
            # Option hash
            attr_reader :options
            # If non-nil, this is a block which, when called, will return the
            # C++ code needed to convert +type+ into +intermediate+. If nil, it
            # is assumed that the user will provide that code.
            attr_reader :code_generator

            # Backtrace at the definition point
            attr_accessor :caller

            def initialize(type, intermediate, options, code_generator)
                if !type || !intermediate
                    raise ArgumentError, "trying to create an opaque definition with nil types"
                end

                @type, @intermediate, @options, @code_generator =
                    type, intermediate, options, code_generator
            end
            # The set of paths that should be added to -I to the generated
            # +type+ to +intermediate+ convertion.
            def includes; options[:includes] end
            # If true, the opaque needs to be copied into the intermediate. If
            # false, the convertion does not require a copy.
            def needs_copy?; !!options[:needs_copy] end
            # If true, the convertion function is provided by the user, and
            # orogen should therefore generate the corresponding templates.
            def generate_templates?; !code_generator end
        end

        # This is an internal structure to pass different sets of types to the
        # code generation plugins
        class TypeSets
            # The types that are new in this typekit
            attr_accessor :types
            # The types for which a convertion function should be generated,
            # except for arrays that have to be handled separately
            attr_accessor :converted_types
            # The array types for which a convertion function should be
            # generated
            attr_accessor :array_types
            # The types that will end up being registered in the type system
            attr_accessor :registered_types
            # The types that can be used on task interfaces. They are different
            # from +registered_types+ as XML marshalling (for instance) need all
            # types to be registered, even the ones that can't be used in the
            # interfaces
            attr_accessor :interface_types
            #
            attr_accessor :minimal_registry
            #
            attr_accessor :opaque_types
            attr_accessor :aliases
        end

        # Support for typekit generation in oroGen
	class Typekit
            class << self
                attr_reader :plugins
            end
            @plugins = Hash.new

            def standalone?
                !project
            end
            
            # Register a new plugin class. The plugin name is taken from
            # klass.name
            def self.register_plugin(klass)
                plugins[klass.name] = klass
            end

            # The Project instance this typekit is part of. It may be nil if the
            # Typekit is generated standalone (as, for instance, in typegen)
	    attr_reader :project

            # For backward compatibility only. Replaced by #project
            def component; project end

            # The set of headers loaded by #load, as an array of absolute paths
            attr_reader :loads

            # The base directory. Everything under that directory are expected
            # to be local files and will be installed by the typekit's cmake
            # code
            attr_reader :base_dir

            # Changes the typekit base directory
            def base_dir=(path)
                @base_dir = path
                if path
                    include_dirs << path
                end
            end

            # The directory in which generated files that are meant to not be
            # modified by the user should be saved
            attr_accessor :automatic_dir
            # The directory in which files that the user should modify should be
            # saved
            attr_accessor :user_dir
            # The directory in which new versions of the user files should be
            # generated. If nil, they will not be generated at all
            attr_accessor :templates_dir
            # The typekit name
            attr_accessor :name
            # The typekit version string
            attr_accessor :version
            # The set of include directories that should be considered in #load
            attr_accessor :include_dirs

            # The set of external headers that have been imported in this
            # typekit. I.e. it contains the headers that are loaded from a
            # depended-upon library for instance.
            def external_loads
                loads.find_all do |name|
                    name !~ /#{Regexp.quote(base_dir)}/
                end
            end

            # The three possible type export policies. See #type_export_policy
            # and #export_types
            TYPE_EXPORT_POLICIES = [:all, :used, :selected]

            ##
            # :method: type_export_policy
            #
            # call-seq:
            #   type_export_policy new_policy => new_policy
            #   type_export_policy => current_policy
            #
            # Change or read the current type export policy. This policy drives
            # what types orogen will import in the RTT type system.
            #
            # If the :all policy is used (the default), then all types that have
            # been imported through header files will be included. This can
            # generate a lot of code, and therefore produce high compilation
            # times and big typekit libraries.
            #
            # If the :used policy is used, then will only be exported the types
            # that are actually used in a task context of the typekit's project.
            # This is most of the time a good idea.
            #
            # If the :selected policy is used, then only types that have been
            # explicitely selected with #export_types will be included.
            dsl_attribute :type_export_policy do |new_policy|
                new_policy = new_policy.to_sym
                if !TYPE_EXPORT_POLICIES.include?(new_policy)
                    raise ArgumentError, "invalid type export policy #{new_policy.inspect}, allowed are: :#{TYPE_EXPORT_POLICIES.join(", :")}"
                end

                if new_policy == :used && !project
                    raise ArgumentError, "cannot use a 'used' policy on a standalone typekit"
                end

                new_policy
            end

            # Select a set of types to be exported through the RTT type system,
            # instead of exporting everything. This is meant to reduce the
            # typekit's code size and compilation times tremendously.
            #
            # See also #type_export_policy
            def export_types(*selection)
                @selected_types |= selection.map { |name| find_type(name) }.to_value_set
            rescue Typelib::NotFound => e
                raise ConfigError, e.message, e.backtrace
            end

            attr_reader :selected_types

	    attr_reader :registry
            attr_reader :imported_types
            attr_accessor :imported_typelist
            attr_reader :imported_typekits

            attr_reader :used_libraries
            attr_reader :linked_used_libraries

            # Returns the set of opaque definitions that are known by the
            # typekit
            attr_reader :opaques

            # Returns the set of opaques that have been added to this particular
            # typekit
            def self_opaques
                opaques.find_all { |opdef| !imported_type?(opdef.type) }
            end

            attr_reader :opaque_registry
            attr_reader :pending_load_options
            attr_reader :pending_loads

            def using_typekit(typekit)
                self.imported_types.merge(typekit.registry)
                self.imported_typelist |= typekit.typelist
                self.include_dirs      |= typekit.include_dirs.to_set
                self.opaques.concat(typekit.opaques)
                if dir = typekit.types_dir
                    self.include_dirs << dir
                end
                self.imported_typekits << typekit
            end

            def using_library(library, options = Hash.new)
                options = Kernel.validate_options options, :link => true
                self.used_libraries << library
                self.include_dirs |= library.include_dirs.to_set
                if options[:link]
                    self.linked_used_libraries << library
                end
            end

            # Returns the typekit object that defines this type
            def imported_typekits_for(typename)
		if typename.respond_to?(:name)
		    typename = typename.name
		end
                return imported_typekits.find_all { |tk| tk.includes?(typename) }
            end

            # Returns true if +typename+ has been defined by a typekit imported
            # by using_typekit
            def imported_type?(typename)
                !imported_typekits_for(typename).empty?
            end

            # Returns the Typelib::Type subclass that represents the type whose
            # name is given. If the type is a derived type (pointer, array or
            # container), then it will be built on the fly.
	    def find_type(type, is_normalized = false)
                if type.respond_to?(:to_str)
                    type_name = type.gsub('::', '/')
                    if !is_normalized
                        type_name = Typelib::Type.normalize_typename(type_name)
                        is_normalized = true
                    end
                    begin
                        registry.build(type_name)
                    rescue Typelib::NotFound
                        if type_name =~ /(.*)\[(\d+)\]$/
                            base_type, array_size = $1, $2
                            find_type(base_type, true)
                            return registry.build(type_name)
                        end

                        container_name, template_arguments = Typelib::GCCXMLLoader.parse_template(type_name)
                        if template_arguments.size == 1
                            element_type = find_type(template_arguments[0], true)
                            if project
                                project.registry.define_container(container_name,
                                                project.find_type(element_type.name, true))
                            end
                            return registry.define_container(container_name, element_type)

                        elsif project && type = project.registry.build(type_name)
                            while type.respond_to?(:deference)
                                type = type.deference
                            end

                            type_def = project.registry.minimal(type.name)
                            registry.merge(type_def)
                            return registry.build(type_name)
                        end
                    end
                elsif type.kind_of?(Class) && type <= Typelib::Type
                    if !registry.include?(type.name)
                        type_def = type.registry.minimal(type.name)
                        registry.merge(type_def)
                        if project
                            project.registry.merge(type_def)
                        end
                    end
                    return registry.get(type.name)
                else
                    raise ArgumentError, "expected a type object or a type name, but got #{type} (#{type.class})"
                end

            rescue Typelib::NotFound => e
                if !pending_loads.empty?
                    perform_pending_loads
                    retry
                end
                raise e.class, e.message, e.backtrace
	    end

            # The target operating system for orocos. Uses the OROCOS_TARGET
            # environment variable, if set, and defaults to gnulinux otherwise.
            def orocos_target
                Orocos::Generation.orocos_target.dup
            end

            # True if the orocos target is gnulinux
            def linux?; orocos_target == 'gnulinux' end
            # True if the orocos target is xenomai
            def xenomai?; orocos_target == 'xenomai' end

	    # Set of directories in which the header files that have been
	    # loaded lie. It is used to generate the Cflags: line in the
	    # pkg-config file
	    attr_reader :loaded_files_dirs

	    def initialize(project = nil)
		@project = project

                @include_dirs = Set.new
                @include_dirs << "/usr/include" << "/usr/local/include"

                @plugins = []
                plugins << (Orocos::TypekitMarshallers::TypeInfo::Plugin.new)

                @internal_dependencies = []
		@imports, @loads    = [], []
		@registry           = Typelib::Registry.new
		@imported_typekits  = []
		@imported_types     = Typelib::Registry.new
                @imported_typelist  = Set.new

                @used_libraries        = []
                @linked_used_libraries = []

		@opaque_registry    = Typelib::Registry.new
                @opaques            = Array.new
		@loaded_files_dirs  = Set.new
                @pending_load_options = []
                # The order matters ! GCCXML unfortunately only gives as file
                # names the argument to #include. So, if we are unlucky, one
                # file will be loaded recursively and we won't actually detect
                # it ... :(
                #
                # In other words, keep pending_loads an array
                @pending_loads        = Array.new

                type_export_policy :all
                @selected_types = ValueSet.new
                @excluded_types = ValueSet.new
                Project.using_rtt_typekit(self)
	    end

            # The set of code generation plugins
            attr_reader :plugins

            def self.transport_plugin_name(transport_name, typekit_name)
                plugins.each_value do |plg|
                    if plg.name == transport_name
                        if plg.respond_to?(:plugin_name)
                            return plg.plugin_name(typekit_name)
                        else
                            return "orogen_typekits::#{typekit_name}#{transport_name.capitalize}TransportPlugin"
                        end
                    end
                end
                raise ArgumentError, "invalid transport name #{transport_name}"
            end

            # Add a generation plugin to the generation stage
            def enable_plugin(name)
                if plugins.any? { |plg| plg.name == name }
                    # It is already there
                    return
                end

                if !(plugin = Typekit.plugins[name])
                    raise ArgumentError, "there is not typekit plugin called #{name}"
                end
                plugins << plugin.new
            end

            def plugin(name)
                if plg = plugins.find { |p| p.name == name }
                    return plg
                else
                    raise ArgumentError, "there is no plugin called #{name}"
                end
            end

            # Enumerate all enabled plugins. See #enable_plugin
            def each_plugin(&block); plugins.each(&block) end

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
            # The Typekit#ro_ptr and Typekit#shared_ptr shortcuts are defined
            # for boost::shared_ptr and RTT::ReadOnlyPointer.
            def smart_ptr(name, base_type, options = Hash.new)
                typekit = self
                opaque_type("#{name}<#{base_type.name}>", base_type, options.merge(:needs_copy => false)) do |from, into|
                    Generation.render_template('typekit/smart_ptr.cpp', binding)
                end
            end

            # Make the typekit define the specialization of RTT::ReadOnlyPointer
            # for the given type.
            #
            # See #smart_ptr for more information.
            def ro_ptr(name, options = Hash.new)
                options[:includes] ||= Array.new
                options[:includes] << 'rtt/extras/ReadOnlyPointer.hpp'
                smart_ptr("/RTT/extras/ReadOnlyPointer", find_type(name), options)
            end

            # Make the typekit define the specialization of boost::shared_ptr
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
            #   void orogen_typekits::toIntermediate(intermediate_type& intermediate, base_type const& sample)
            #   void orogen_typekits::fromIntermediate(base_type& sample, intermediate_type const& intermediate)
            #
            # In the first examples above, this would be (assuming an "imu" orogen project)
            #
            #   void orogen_typekits::toIntermediate(imu::Vector3f& intermediate, Eigen::Vector3f const& sample)
            #   void orogen_typekits::fromIntermediate(Eigen::Vector3f& sample, imu::Vector3f& intermediate)
            #
            # Note that in fromIntermediate, the +intermediate+ argument in
            # non-const. It is actually allows to modify it, as for instance to
            # get better performance (example: use +vector<>.swap()+ instead of
            # doing a big copy).
            #
            # If +:needs_copy+ is false, then we assume that a copy is not
            # needed. In that case, the toIntermediate convertion method will
            # return the intermediate type directly. The signature will
            # therefore be changed to
            #
            #   intermediate_type& orogen_typekits::toIntermediate(base_type const& sample)
            #   bool orogen_typekits::fromIntermediate(base_type& sample, intermediate_type* intermediate)
            #
            # Note that in fromIntermediate the +intermediate+ argument is now
            # given as a non-const pointer.  The convertion function can choose
            # to take ownership on that value, in which case it has to return
            # true. If the function returns false, then the sample is deleted after
            # the method call.
            def opaque_type(base_type, intermediate_type, options = {}, &convert_code_generator)
                options = validate_options options,
                    :includes => [],
                    :needs_copy => true

                if options[:includes].respond_to?(:to_str)
                    options[:includes] = [options[:includes]]
                end

                base_type = base_type.to_str
                base_type = Typelib::Type.normalize_typename(base_type)
                if intermediate_type.kind_of?(Class) && intermediate_type < Typelib::Type
                    intermediate_type = intermediate_type.name
                else
                    intermediate_type = Typelib::Type.normalize_typename(intermediate_type)
                end

                typedef = "<typelib><opaque name=\"#{base_type.gsub('<', '&lt;').gsub('>', '&gt;')}\" size=\"#{0}\" /></typelib>"
                opaque_def = Typelib::Registry.from_xml(typedef)
                opaque_registry.merge opaque_def
                registry.merge opaque_def
                if project
                    project.registry.merge opaque_def
                end

                opaque_type = find_type(base_type, true)
                orogen_def  = OpaqueDefinition.new(opaque_type,
                                                 intermediate_type, options, convert_code_generator) 
                orogen_def.caller = caller
                @opaques << orogen_def
                @opaques = opaques.
                    sort_by { |orogen_def| orogen_def.type.name }
                opaque_type.name
            end

            # True if there are some opaques in this typekit. The result of this
            # method is only valid during generation. Don't use it in general.
            def has_opaques?
                self_types.any? { |t| t.contains_opaques? }
            end

            # True if some opaques require to generate templates
            def has_opaques_with_templates?
                self_opaques.any? { |op| op.generate_templates? }
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
	    def load(file, add = true, user_options = Hash.new)
                if !user_options.respond_to?(:to_hash) 
                    raise ArgumentError, "expected an option has as third argument, got #{user_options.inspect}"
                end

                if match = /(\w+)\/Types.hpp$/.match(file)
                    project_name = match[1]
                    if project.has_typekit?(project_name) || project.name == project_name
                        raise ArgumentError, "cannot use a header called #{file} as it clashes with the Types.hpp header generated by orogen for the #{project_name} project"
                    end
                end

                this_options = [add, user_options]
                if pending_load_options != this_options
                    perform_pending_loads
                end

                include_dirs = self.include_dirs

                # Get the full path for +file+
                file =
                    if File.exists?(file) # Local file
                        File.expand_path(file)
                    else # File from used libraries/task libraries
                        dir = include_dirs.find { |dir| File.exists?(File.join(dir, file)) }
                        if !dir
                            raise ArgumentError, "cannot find #{file} in #{include_dirs.to_a.join(":")}"
                        end
                        loaded_files_dirs << dir
                        File.join(dir, file)
                    end

                @pending_load_options = this_options
                pending_loads << file
            end

            def filter_unsupported_types(registry)
                to_delete = Set.new
                registry.each do |type|
                    # Multi-dimensional arrays are forbidden in CORBA IDL
                    # TODO: work around this in Typelib and oroGen by emitting a
                    # TODO: single-dimension array of the right size
                    if type < Typelib::ArrayType && type.deference < Typelib::ArrayType
                        STDERR.puts "WARN: ignoring #{type.name} as multi-dimensional arrays cannot be represented in CORBA IDL"
                        to_delete << type

                    elsif type < Typelib::PointerType
                        STDERR.puts "WARN: ignoring #{type.name} as pointers are not allowed"
                        to_delete << type

                    elsif type < Typelib::CompoundType
                        type.each_field do |field_name, _|
                            if field_name !~ /^[a-zA-Z]/
                                STDERR.puts "WARN: ignoring #{type.name} as its field #{field_name} does not start with an alphabetic character, which is forbidden in CORBA IDL"
                                to_delete << type
                                break
                            end
                        end
                    end
                end

                already_deleted = to_delete.dup
                to_delete.each do |type|
                    deleted_types = registry.remove(type)
                    deleted_types.each do |dep_type|
                        next if to_delete.include?(dep_type)
                        STDERR.puts "WARN: ignoring #{dep_type.name} as it depends on #{type.name} which is ignored"
                    end
                end
            end

            def perform_pending_loads
                return if pending_loads.empty?

                add, user_options = *pending_load_options

                include_dirs = self.include_dirs.to_a
                if automatic_dir
                    include_dirs << File.join(automatic_dir, "types")
                end

                file_registry = Typelib::Registry.new
                file_registry.merge opaque_registry

                options = { :opaques_ignore => true, :merge => false, :required_files => pending_loads.to_a }
                # GCCXML can't parse vectorized code, and the Typelib internal
                # parser can't parse eigen at all. It is therefore safe to do it
                # here
                options[:define] = ['EIGEN_DONT_VECTORIZE', "OROCOS_TARGET=#{Orocos::Generation.orocos_target}"]

                options[:include] = include_dirs.dup
                options = options.merge(user_options) do |key, a, b|
                    if a.respond_to?(:to_ary)
                        if b.respond_to?(:to_ary)
                            b.concat(a)
                        else
                            [b].concat(a)
                        end
                    else
                        b
                    end
                end

                Tempfile.open("orogen_pending_loads") do |io|
                    pending_loads.each do |f|
                        io.puts "#include \"#{f}\""
                    end
                    io.flush

                    begin
                        do_import(file_registry, io.path, 'c', options)
                        filter_unsupported_types(file_registry)
                        registry.merge(file_registry)
                        if project
                            project.registry.merge(file_registry)
                        end
                    rescue Exception => e
                        raise ArgumentError, "cannot load one of the header files #{pending_loads.join(", ")}: #{e.message}", e.backtrace
                    end
                end

                if add
                    loads.concat(pending_loads.to_a)
                end
                pending_loads.clear
	    end

            # Returns the set of headers that have been loaded locally
            def local_headers(absolute = true)
                list = loads.find_all { |path| path =~ /^#{Regexp.quote(base_dir)}\// }
                if absolute
                    list
                else
                    list.map do |p|
                        relative = p.gsub(/^#{Regexp.quote(base_dir)}\//, '')
                        dest = relative.
                            gsub(/^#{Regexp.quote(Generation::AUTOMATIC_AREA_NAME)}\//, '').
                            gsub(/^#{name}\//, '')

                        [relative, dest]
                    end
                end
            end

            # Packages defined in this typekit on which the typekit should
            # depend. See #internal_dependency.
            attr_reader :internal_dependencies

            # The second one allows to specify a dependency of the typekit on a
            # library defined in the same CMake package. The typekit's .pc file
            # will therefore depend on the specified pkg-config package (among
            # other things). Do that when you define a library of types and
            # want people to be able to use it even though they don't have
            # orogen.
            #
            # In other words, it allows to build packages were:
            #  * a normal C/C++ library is defined/built/install
            #  * an orogen typekit is defined, which wraps the types defined by this library
            def internal_dependency(name, version = nil)
                @internal_dependencies << [name, version]
            end

	    def import(other_typekit)
		raise NotImplementedError
	    end

            BASE_TYPES = ['int', 'unsigned int', 'double', 'float', 'bool', 'char']
            BASE_TYPES_RTT_NAMES = {
                'unsigned int' => 'uint' }
            BASE_TYPES_NEEDED_TRANSPORTS = %w{typelib}

            def normalize_registry(with_base_types = false)
                base = self.registry
                result = Typelib::Registry.new
                self_types.each do |type|
                    result.merge(base.minimal(type.name))
                end

                # Also register the intermediate types for our opaques. The
                # resulting registry must be self-contained, and the
                # intermediate stuff is specific to oroGen (i.e. typelib does
                # not handle them for us)
                opaques.each do |op_def|
                    if result.include?(op_def.type.name)
                        result.merge(base.minimal(find_type(op_def.intermediate).name))
                    end
                end

                if with_base_types
                    standard_types = Typelib::Registry.new
                    Typelib::Registry.add_standard_cxx_types(standard_types)
                    base.merge(standard_types)

                    BASE_TYPES.each do |typename|
                        result.merge(base.minimal(typename))
                    end
                end

                result
            end

            def imported_array_of?(type)
                typename = if type.respond_to?(:name) then type.name
                           else type.to_str
                           end

                imported_typelist.any? { |str| str =~ /#{Regexp.quote(typename)}(\[\d+\])+/ }
            end

            # List of typekits that this typekit depends on
            def used_typekits
                @registry = normalize_registry
                result = Set.new

                # We depend on the typekits that define types that are used in
                # our types (ouch), as they define the convertion functions that
                # our typekit will use.
                #
                # We therefore must not use self_types there but really
                # registry.each_type. Note that the registry is minimal, i.e.
                # contains only our own types plus the types that are needed to
                # define them, and this is therefore not a problem.
		registry.each(:with_aliases => true) do |name, type|
                    loop do
                        imported_typekits.each do |tk|
                            if type < Typelib::ArrayType 
                                if imported_array_of?(type.deference)
                                    result << tk
                                end
                            elsif tk.includes?(type.name)
                                result << tk
                            end
                        end

                        if type.respond_to?(:deference)
                            type = type.deference
                        else break
                        end
                    end
		end

                result.to_a.sort_by { |tk| tk.name }
            end

            # Returns the set of pkg-config packages this typekit depends on
            def dependencies
                result = []

                # We must link to libraries in case the types we are getting
                # from them has constructors/destructors that the library
                # provides.
                used_libraries.each do |pkg|
                    needs_link = linked_used_libraries.include?(pkg)
                    result << BuildDependency.new(pkg.name.upcase, pkg.name).
                        in_context('core', 'include')
                    if needs_link
                        result.last.in_context('core', 'link')
                    end
                end

                # We must link to typekits that define our types, as we are
                # going to reuse the convertion functions that they define
                used_typekits.each do |tk|
                    next if tk.virtual?
                    result << BuildDependency.new(
                        tk.name.upcase + "_TYPEKIT", tk.pkg_name).
                        in_context('core', 'include').
                        in_context('core', 'link')
                end

                # We must include the typekits that define types that are used
                # in the other typekits types
                each_plugin do |plg|
                    if !plg.separate_cmake?
                        if deps = plg.dependencies(self)
                            result.concat(deps)
                        end
                    end
                end

                result.to_a.sort_by { |dep| dep.var_name }
            end

            include OpaqueHandling

            def intermediate_cxxname_for(type_def)
                type = find_type(type_def)
                if type < Typelib::ArrayType
                    prefix, suffix = intermediate_cxxname_for(type.deference)
                    [prefix, "#{suffix}[#{type.length}]"]
                else
                    name = intermediate_type_name_for(type)
                    [name.gsub('/', '::').gsub('<::', '< ::')]
                end
            end


            def m_types_code
                @m_types_code
            end

            def handle_opaques_generation(generated_types)
                # Make sure all opaque intermediate types are existing or can be
                # instanciated
                self_opaques.each do |opaque_def|
                    begin
                        t = find_type(opaque_def.intermediate)
                        if t.contains_opaques?
                            raise ConfigError, "the type #{opaque_def.intermediate} is used as an intermediate type for #{opaque_def.type.name}, but it is an opaque or contains opaques"
                        end
                    rescue Typelib::NotFound
                        raise ConfigError, "type #{opaque_def.intermediate}, used as intermediate for opaque #{opaque_def.type.name}, does not exist", opaque_def.caller
                    end
                end

                # Generate some type definitions for the pocosim marshalling. In
                # practice, we generate C code that we merge back into the
                # repository
                generate_all_marshalling_types = false
                needed_definitions = self_types.
                    find_all { |t| t.contains_opaques? && !t.opaque? }

                needed_types = []
                while !needed_definitions.empty?
                    needed_definitions.delete_if do |type|
                        next if type.dependencies.any? { |t| needed_definitions.include?(t) }
                        needed_types << type
                    end
                end

                options = { :include => include_dirs.dup }

                typekit = self
                catch(:nothing_to_define) do
                    Tempfile.open('orogen') do |io|
                        Orocos::Generation.debug "loading m-type definitions"
                        marshalling_code = Generation.render_template 'typekit', 'marshalling_types.hpp', binding
                        @m_types_code = marshalling_code
                        io << marshalling_code
                        io.flush
                        self.load(io.path, false, options)

                        begin
                            self.perform_pending_loads
                        rescue Exception => e
                            marshalling_code = marshalling_code.split("\n").
                                each_with_index.map do |l, i|
                                    "#{i} #{l}"
                                end.join("\n")
                            raise InternalError, "cannot load the definition of opaque m-types: #{e.message}\nThe faulty code is\n\n#{marshalling_code}", e.backtrace
                        end
                    end
                end
                true
            end

            def do_import(registry, path, kind, options)
                # Define __orogen2 or __orogen based on us using GCCXML (first
                # case) or the typelib internal parser (second case)
                uses_gccxml =
                    Typelib::Registry.respond_to?(:uses_gccxml?) && Typelib::Registry.uses_gccxml?(path, 'c')
                options[:define] ||= []
                if uses_gccxml
                    options[:define] << '__orogen2'
                    opaque_names = opaques.map { |opdef| opdef.type.name }
                    options[:opaques] ||= []
                    options[:opaques].concat(opaque_names)
                else
                    options[:define] << '__orogen'
                end

                registry.import(path, kind, options)
            end

            # This generates typedefs for container types. These
            # typedefs are needed because IDL and the CORBA C++ mapping do not
            # allow to reference sequence types directly (you have to typedef
            # them first, or use them in a compound)
            def generate_typedefs(generated_types, registry)
            end

	    def issue_warnings(generated_types, registry)
		generated_types.each do |type|
		    if type.contains_int64?
			Orocos::Generation.warn "you will not be able to marshal #{type.name} as XML, it contains 64bit integers"
		    end
		end
	    end

            # Returns the set of type names defined in this typekit. This is
            # different from self_types, as it returns a set of type names (i.e.
            # strings), and also because it includes the aliases defined by the
            # typekit.
            def self_typenames
		generated_types = []
		registry.each(:with_aliases => true) do |name, type|
                    next if imported_type?(name)
		    if !type.inlines_code?
			generated_types << name
		    end
		end
                generated_types
            end

            # Returns the set of types that are specifically handled by this
            # typekit
            def self_types
		generated_types = []
		registry.each do |type|
                    next if imported_type?(type.name)
                    generated_types << type
		end
                generated_types
            end

            # Helper method that returns the code needed to get an
            # +intermediate+ variable of the right type, containing the data in
            # +value+.
            def code_toIntermediate(intermediate_type, needs_copy, indent)
                if intermediate_type < Typelib::ArrayType
                    "#{indent}std::vector< #{intermediate_type.deference.cxx_name} > intermediate;\n" +
                    "#{indent}intermediate.resize(length);\n" +
                    "#{indent}orogen_typekits::toIntermediate(&intermediate[0], value, length);\n"
                elsif needs_copy
                    "#{indent}#{intermediate_type.cxx_name} intermediate;\n" +
                    "#{indent}orogen_typekits::toIntermediate(intermediate, value);\n"
                else
                    "#{intermediate_type.cxx_name} const& intermediate = orogen_typekits::toIntermediate(value);"
                end
            end

            # Helper method that returns the code needed to update an opaque
            # type based on the data from an intermediate variable
            def code_fromIntermediate(intermediate_type, needs_copy, indent)
                if intermediate_type < Typelib::ArrayType
                    "#{indent}orogen_typekits::fromIntermediate(value, length, &intermediate[0]);\n"
                elsif needs_copy
                    "#{indent}orogen_typekits::fromIntermediate(value, intermediate);\n"
                else
                    "#{indent}if (orogen_typekits::fromIntermediate(value, intermediate.get()))\n" +
                    "#{indent}    intermediate.release();\n"
                end
            end

            attr_reader :template_instanciation_files

            def save_automatic(*args)
                Generation.save_generated(true, automatic_dir, *args)
            end

            def save_user(*args)
                path = Generation.save_generated(false, user_dir, *args)
                if templates_dir
                    Generation.save_generated(true, templates_dir, *args)
                end
                path
            end

            def transport_plugin_name(transport_name)
                self.class.transport_plugin_name(transport_name, self.name)
            end

            def render_typeinfo_snippets(code_snippets, *place)
                return [] if code_snippets.empty?

                impl = []
                place = File.join(*place)
                code_snippets.each do |code|
                    if code[0].respond_to?(:name_as_word)
                        code[0] = code[0].name_as_word
                    else
                        code[0] = code[0].to_str
                    end
                end
                code_snippets = code_snippets.sort_by { |code| code[0] }

                slice_size = Orocos::Generation.typekit_slice
                while slice_size > 1 && code_snippets.size / slice_size < Orocos::Generation.typekit_slice_minimum
                    slice_size -= 1
                end

                code_snippets.each_slice(slice_size) do |code|
                    file_name = code.map(&:first).join("_")
                    code = code.map(&:last).join("\n")
                    impl << save_automatic(place, "#{file_name}.cpp", code)
                end
                impl
            end

            # Makes sure that a set of type objects comes from the same registry
            #
            # In order to use ValueSet, we must make sure that all type objects
            # come from the same registry. This method takes a ValueSet of types
            # and converts all of them to the type coming from +registry+
            def map_typeset_to_registry(registry, types)
                types.map { |t| find_type(t) }.to_value_set
            end

	    def generate
		typekit = self

                FileUtils.mkdir_p automatic_dir

                # Populate a fake installation directory so that the include
                # files can be referred to as <project_name>/header.h
                fake_install_dir = File.join(automatic_dir, name)

                # Upgrade from directory to symlink
                if !File.symlink?(fake_install_dir)
                    FileUtils.rm_rf fake_install_dir
                end
                Generation.create_or_update_symlink(automatic_dir, fake_install_dir)

                if standalone?
                    fake_typekit_dir = File.join(automatic_dir, "typekit")
                    Generation.create_or_update_symlink(automatic_dir, fake_typekit_dir)
                end

                # Small hack to workaround the current structure w.r.t. our
                # usage of the main RTT typekit
                #
                # We need to generate the typelib / mqueue transports (not
                # CORBA) for the base types *if and only if* we are NOT
                # standalone (we are using oroGen and not typegen) *and* the
                # oroGen project does not import any other typekit
                generate_transports_for_base_types = (!standalone? && project.used_typekits.find_all { |tk| !tk.virtual? }.empty?)

                # Load any queued file. This must be done before the call
                # to local_headers below, as the new files will get
                # registered only after the call
                perform_pending_loads

                self.local_headers(false).each do |path, dest_path|
                    dest_path = File.join(automatic_dir, "types", name, dest_path)
                    Generation.create_or_update_symlink(File.join(base_dir, path), dest_path)
                end

                # Generate opaque-related stuff first, so that we see them in
                # the rest of the typelib-registry-manipulation code
                handle_opaques_generation(registry)

		# Do some registry mumbo-jumbo to remove unneeded types to the
                # dumped registry
                @registry = normalize_registry(generate_transports_for_base_types)
		minimal_registry = @registry.dup
                generated_types = self_types.to_value_set

                self_opaques = self.self_opaques.sort_by { |opdef| opdef.type.name }
                opaque_types = self_opaques.map { |opdef| opdef.type }
                opaque_intermediates = self_opaques.map do |opdef|
                    find_type(opdef.intermediate)
                end

		issue_warnings(generated_types, minimal_registry)
                generate_typedefs(generated_types, minimal_registry)

                # Generate the XML representation of the generated type library,
                # and add opaque information to it
                plain_registry = minimal_registry.to_xml
                doc = Nokogiri::XML(plain_registry)
                doc.xpath('//opaque').each do |opaque_entry|
                    spec = opaque_specification(opaque_entry['name'])

                    opaque_entry['marshal_as'] = spec.intermediate
                    opaque_entry['includes']   = spec.includes.join(':')
                    opaque_entry['needs_copy'] = (spec.needs_copy? ? '1' : '0')
                end
                save_automatic "#{name}.tlb", doc.to_xml

                # The first array is the set of types for which convertion
                # functions are generated. The second is the set of types that
                # are actually registered into the RTT type system
                #
                # The two arrays are sorted so that we don't have to recompile
                # unnecessarily (the original sets are hashes, and therefore don't
                # have a stable order).
                converted_types = generated_types.
                    sort_by { |type| type.name }

                # We need a special case for arrays. The issue is the following:
                # for arrays, the convertion functions take pointers as input.
                # So we generate only one convertion function for all arrays of
                # the same type, regardless of the size.
                converted_types, array_types = converted_types.
                    partition { |type| !(type < Typelib::ArrayType) }

                array_types = array_types.
                    delete_if do |type|
                        imported_array_of?(type.deference)
                    end.
                    inject(Hash.new) { |h, type| h[type.deference.name] = type; h }.
                    values.
                    sort_by { |type| type.name }

                # If the selected type export policy is used, check if it makes
                # sense. If it does not, switch back to 'all'
                if type_export_policy == :used && (!project || project.self_tasks.empty?)
                    type_export_policy :all
                end

                # Get the types that should be registered in the RTT type
                # system. Note that we must map these types to +registry+ as
                # +registry+ has been recomputed (i.e. it changes between the
                # specification time and the generation time)
                #
                # Only +generated_types+ does not need to be mapped as it has
                # just been computed
                registered_types =
                    if type_export_policy == :all
                        generated_types.dup
                    elsif type_export_policy == :used
                        used_types = project.self_tasks.inject(ValueSet.new) do |result, task|
                            result | map_typeset_to_registry(registry, task.interface_types)
                        end
                        (used_types & generated_types)
                    elsif type_export_policy == :selected
                        map_typeset_to_registry(registry, selected_types)
                    end

                if !selected_types.empty?
                    registered_types |= map_typeset_to_registry(registry, selected_types)
                end

                # Save all the types that this specific typekit handles
                registered_typenames = registered_types.map(&:name).to_set
                typelist_txt = []
                generated_types.each do |type|
                    typename = type.name
                    typelist_txt << "#{typename} #{registered_typenames.include?(typename) ? '1' : '0'}"
                end
                save_automatic "#{name}.typelist",
                    typelist_txt.sort.join("\n")

                registered_types = registered_types.
                    sort_by { |t| t.name }
                registered_types.delete_if do |t|
                    t.contains_opaques? && t < Typelib::ArrayType
                end
                interface_types = registered_types.
                    find_all { |t| !(t < Typelib::ArrayType) }

                type_sets = TypeSets.new
                type_sets.types            = generated_types.sort_by { |t| t.name }
                type_sets.converted_types  = converted_types
                type_sets.array_types      = array_types
                type_sets.registered_types = registered_types
                type_sets.interface_types  = interface_types
                type_sets.minimal_registry = minimal_registry
                type_sets.opaque_types     = self_opaques
                type_sets.aliases          = Hash.new

                public_header_files, implementation_files = [], []

		type_header = Generation.render_template('typekit/Types.hpp', binding)
                # Types.hpp is not registered in public_header_files as it gets
                # installed in TYPEKIT_NAME/ directly instead of
                # TYPEKIT_NAME/typekit
		save_automatic("Types.hpp", type_header)
                boost_serialization = Generation.render_template "typekit/BoostSerialization.hpp", binding
                public_header_files << save_automatic("type_info", "BoostSerialization.hpp", boost_serialization)
                tk_hpp = Generation.render_template "typekit/Plugin.hpp", binding
		public_header_files << save_automatic("Plugin.hpp", tk_hpp)
                tk_cpp = Generation.render_template "typekit/Plugin.cpp", binding
		implementation_files << save_automatic("Plugin.cpp", tk_cpp)

                # Generate the opaque convertion files
                if has_opaques?
                    intermediates_hpp = Generation.render_template 'typekit/OpaqueConvertions.hpp', binding
                    public_header_files <<
                        save_automatic("OpaqueConvertions.hpp", intermediates_hpp)

                    intermediates_cpp = Generation.render_template 'typekit/OpaqueConvertions.cpp', binding
                    implementation_files <<
                        save_automatic("OpaqueConvertions.cpp", intermediates_cpp)

                    if has_opaques_with_templates?
                        user_hh = Generation.render_template 'typekit/Opaques.hpp', binding
                        user_cc = Generation.render_template 'typekit/Opaques.cpp', binding
                        save_user("Opaques.hpp", user_hh)
                        implementation_files <<
                            save_user("Opaques.cpp", user_cc)

                        Generation.create_or_update_symlink(File.join(user_dir, "Opaques.hpp"), File.join(automatic_dir, "Opaques.hpp"))
                    end
                end

                if generate_transports_for_base_types
                    base_type_aliases = Hash.new
                    base_types = BASE_TYPES.map do |typename|
                        type = minimal_registry.get(typename)
                        base_type_aliases[type] = [BASE_TYPES_RTT_NAMES[typename] || typename]
                        type
                    end.to_value_set
                end

                each_plugin do |plg|
                    plg_typesets = type_sets.dup
                    if generate_transports_for_base_types && BASE_TYPES_NEEDED_TRANSPORTS.include?(plg.name)
                        plg_typesets.interface_types = (plg_typesets.interface_types.to_value_set | base_types).
                            sort_by { |t| t.name }

                        plg_typesets.aliases = plg_typesets.aliases.merge(base_type_aliases)
                    end

                    headers, impl = plg.generate(self, plg_typesets)
                    public_header_files.concat(headers)
                    implementation_files.concat(impl)
                end

                if standalone?
                    FileUtils.mkdir_p File.join(Generation::AUTOMATIC_AREA_NAME, 'config')
                    Dir.glob File.join(Generation.template_path('config'), '*') do |path|
                        basename    = File.basename(path)
                        if !Project::CMAKE_GENERATED_CONFIG.include?(basename)
                            save_automatic 'config', basename, File.read(path)
                        end
                    end
                end

		pkg_config = Generation.render_template 'typekit/typekit.pc', binding
		save_automatic("#{name}-typekit.pc.in", pkg_config)
                cmake = Generation.render_template 'typekit', 'CMakeLists.txt', binding
                save_automatic("CMakeLists.txt", cmake)
                manifest = Generation.render_template 'typekit', 'manifest.xml', binding
                save_automatic("manifest.xml", manifest)
                makefile = Generation.render_template 'typekit', 'Makefile', binding
                save_automatic("Makefile", makefile)

                # Finished, create the timestamp file
                Generation.cleanup_dir(automatic_dir)
                Generation.touch File.join(automatic_dir, 'stamp')
	    end

            def cmake_relative_path(file, *subdir)
                "${CMAKE_CURRENT_SOURCE_DIR}/#{Pathname.new(file).relative_path_from(Pathname.new(File.join(automatic_dir, *subdir)))}"
            end
	end
    end
end

require 'orogen/marshallers/typelib'
require 'orogen/marshallers/corba'
require 'orogen/marshallers/type_info'
require 'orogen/marshallers/mqueue'
