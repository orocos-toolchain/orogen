# -*- coding: utf-8 -*-
require 'typelib'
require 'tempfile'
require 'find'
require 'orogen/base'
require 'utilrb/kernel/options'

module Typelib
    class Type
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
#{indent}        RTT::log(RTT::Error) << "invalid value '" << (int)value << "' for enum '#{cxx_name}'" << RTT::endlog();
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
    end
end

module OroGen
    module Gen
    module RTT_CPP
        # Helper method used to create a symbolic link. If a link already
        # exists, it makes sure that it is up to date
        def self.create_or_update_symlink(source, target)
            if File.exists?(target)
                if !File.symlink?(target)
                    raise ConfigError, "#{target} was expected to be a symbolic link, but is not"
                end
                pointed_to = File.readlink(target)
                if pointed_to == target
                    return 
                else
                    FileUtils.rm_f target
                end
            end
            FileUtils.mkdir_p(File.dirname(target))
            FileUtils.ln_sf(source, target)
        end

        # Common implementation of opaque-related methods. It gets included in
        # Project and Typekit
        module OpaqueHandling
            # Get the opaque definition for a given type
            #
            # @param [Type,String] the type or type name
            # @return [OpaqueDefinition]
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

            # Finds the opaque (or opaque-containing) type for which the given
            # type is an intermediate
            #
            # @return [Type,nil] the type, or nil if 'type' is not used as an
            #   intermediate
            def find_opaque_for_intermediate(type)
                type = find_type(type.name)
                if m_type?(type)
                    # Yuk
                    begin
                        if @intermediate_to_opaque && (result = @intermediate_to_opaque[type.name])
                            result
                        elsif type.name =~ /_m$/
                            find_type(type.name.gsub(/_m$/, ''))
                        else raise Typelib::NotFound
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
                elsif opaque_def = opaques.find { |spec| find_type(spec.intermediate, true) == type }
                    opaque_def.type
                end
            end

            # Computes the name of the type that should be used as an
            # intermediate for the given type
            #
            # @param [String,Type] type_def the type or type name
            # @param [Boolean] is_normalized if true, the provided type name is
            #   supposed to be normalized. Otherwise, an (expensive) normalization
            #   will be computed if it cannot be found as-is in the typekit's
            #   registry
            # @return [String] the normalized name of the intermediate type, or
            #   the type's name if 'type' is not an opaque
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

            # Gets the intermediate type for a given type
            #
            # @param [Type,String] type_def the type or type name
            # @return [Type] the type of the intermediate, or 'type_def' itself
            #   if 'type_def' is not an opaque
            # @raises Typelib::NotFound if the expected intermediate type cannot
            #   be found
            def intermediate_type_for(type_def)
                typename = intermediate_type_name_for(type_def)
                return find_type(typename, true)
            end

            # Checks if a type is used as an intermediate
            def intermediate_type?(type)
                !!find_opaque_for_intermediate(type)
            end

            # Checks if a type is an oroGen-generated type used as an
            # intermediate
            def m_type?(type)
                typename = type.name
                if type.name =~ /_m$/
                    return true
                end

                if type.respond_to?(:deference)
                    while type.respond_to?(:deference)
                        type = type.deference
                    end
                    m_type?(type)
                else
                    false
                end
            end
        end

        class << self
            # Multiple type info objects can be grouped in a single file to
            # reduce the overhead, at compilation time, of reading the
            # preprocessed file
            #
            # This controls the size of the grouping
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
            def includes; options[:include] end
            # If true, the opaque needs to be copied into the intermediate. If
            # false, the convertion does not require a copy.
            def needs_copy?; !!options[:needs_copy] end
            # If true, the conversion function is provided by the user, and
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

            INCLUDE_DIR_NAME = 'types'

            # Change the directory into which the code generation should be done
            def automatic_dir=(path)
                @automatic_dir = path
                if path
                    include_dirs << File.join(automatic_dir, INCLUDE_DIR_NAME)
                end
            end

            # Full path to the directory in which includes are either generated
            # or symlinked
            def includes_dir
                File.join(automatic_dir, INCLUDE_DIR_NAME)
            end

            # The directory in which generated files that are meant to not be
            # modified by the user should be saved
            attr_reader :automatic_dir
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
            # The array of includes that have been loaded
            attr_reader :included_files

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
                if library.respond_to?(:to_str)
                    library = Utilrb::PkgConfig.new(library)
                end

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
                !selected_types.find { |t| t.name == typename } && !imported_typekits_for(typename).empty?
            end

            # Returns true if +typename+ can be used on a task context interface
            def exported_type?(typename)
                imported_typekits_for(typename).any? do |tk|
                    tk.interface_type?(typename)
                end
            end

            def build_type(type_name)
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
                        project.registry.create_container(container_name,
                                        project.find_type(element_type.name, true))
                    end
                    return registry.create_container(container_name, element_type)

                elsif project && type = project.registry.build(type_name)
                    while type.respond_to?(:deference)
                        type = type.deference
                    end

                    type_def = project.registry.minimal(type.name)
                    registry.merge(type_def)
                    return registry.build(type_name)
                end
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
                        registry.get(type_name)
                    rescue Typelib::NotFound
                        new_type = build_type(type_name)
                        compute_orogen_include_on_type(new_type, Hash.new)
                        return new_type
                    end
                elsif type.kind_of?(Class) && type <= Typelib::Type
                    if !registry.include?(type.name)
                        type_def = type.registry.minimal(type.name)
                        registry.merge(type_def)
                        if project
                            project.registry.merge(type_def)
                        end
                        new_type = registry.get(type.name)
                        compute_orogen_include_on_type(new_type, Hash.new)
                        return new_type
                    else
                        return registry.get(type.name)
                    end
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
                RTT_CPP.orocos_target.dup
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
                @included_files = Array.new

                @plugins = []
                plugins << (TypekitMarshallers::TypeInfo::Plugin.new(self))

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
                # The order matters ! GCCXML unfortunately gave as file names
                # the argument to #include. So, if we are unlucky, one file
                # will be loaded recursively and we won't actually detect it
                # ... :( Not sure if this is still relevant with the
                # clang-based importer
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
            #
            # @return [Object] the newly created plugin
            def enable_plugin(name)
                if plugins.any? { |plg| plg.name == name }
                    # It is already there
                    return
                end

                if !(plugin = Typekit.plugins[name])
                    raise ArgumentError, "there is not typekit plugin called #{name}"
                end
                p = plugin.new(self)
                plugins << p
                p
            end


            # Looks for an enabled plugin called +name+
            #
            # @param [String] name the name of the plugin to look for
            # @return [Object] the plugin object, or nil if it cannot be found
            # @see plugin
            def find_plugin(name)
                return plugins.find { |p| p.name == name }
            end

            # Return the plugin object for +name+
            #
            # @param [String] name the name of the plugin to look for
            # @return [Object] the plugin object, or nil if it cannot be found
            # @raise [ArgumentError] raised if no plugin is enabled with name
            # +name+
            # @see find_plugin
            def plugin(name)
                if plg = find_plugin(name)
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
            def smart_ptr(ptr_name, base_type, options = Hash.new)
                typekit = self
                # create a validated options-hash
                options = Kernel.validate_options options,
                    # just the plain name of the header like "path/Header.hpp"
                    :include => [],
                    # additional pkg-config info like "pkg-config-name:path/Header.hpp"
                    :orogen_include => []
                # create opaque
                opaque = opaque_type("#{ptr_name}<#{base_type.name}>",
                                     base_type,
                                     options.merge(:needs_copy => false)) do |from, into|
                    # create code from template
                    Generation.render_template('typekit/smart_ptr.cpp', binding)
                end
                if base_type_inc = existing_orogen_include_for_type(base_type)
                    base_type_inc.each do |or_inc|
                        opaque.metadata.add('orogen_include', or_inc)
                    end
                end

                # handle the "orogen_include" option, which has to contain the
                # pkg-config-name of the software providing this header
                options[:orogen_include].each do |or_inc|
                    included_files << or_inc.split(':').last
                    opaque.metadata.add('orogen_include', or_inc)
                end
                # Add the headers required for this smart pointer definition to
                # the list of included files. we dont have any pkg-config
                # informations in this channel, so just prepend a ":" to denote
                # the empty case.
                options[:include].each do |inc|
                    included_files << inc
                    # note that the "opaque_type()" factory-function used
                    # previously to create the opaque for the ro_ptr will take
                    # care that this header is added to the "orogen_include"
                    # metadata as well
                end
                # ...and copy the header-informations needed for the type
                # verbatim.
                base_type.metadata.get('orogen_include').each do |or_inc|
                    opaque.metadata.add('orogen_include', or_inc)
                end
                opaque
            end

            # Make the typekit define the specialization of RTT::ReadOnlyPointer
            # for the given type.
            #
            # See #smart_ptr for more information.
            def ro_ptr(name, options = Hash.new)
                options[:orogen_include] ||= Array.new
                options[:orogen_include] << "orocos-rtt-#{RTT_CPP.orocos_target}:rtt/extras/ReadOnlyPointer.hpp"
                smart_ptr("/RTT/extras/ReadOnlyPointer", find_type(name), options)
            end

            # Make the typekit define the specialization of boost::shared_ptr
            # for the given type.
            #
            # See #smart_ptr for more information.
            def shared_ptr(name, options = Hash.new)
                options[:orogen_include] ||= Array.new
                # actually we would need to add a pkg-config name in front of
                # the ":", but boost does _still_ not have pkg-config...
                options[:orogen_include] << ':boost/shared_ptr.hpp'
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
            def opaque_type(base_type, intermediate_type, options = Hash.new, &convert_code_generator)
                options = Kernel.validate_options options,
                    :include => [],
                    :includes => [],
                    :orogen_include => [],
                    :needs_copy => true

                if options[:includes].respond_to?(:to_str)
                    options[:includes] = [options[:includes]]
                end
                if options[:include].respond_to?(:to_str)
                    options[:include] = [options[:include]]
                end
                options[:include].concat(options.delete(:includes))

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
                options[:include].each do |inc|
                    opaque_type.metadata.add('orogen_include', "#{self.name}:#{inc}")
                end
                orogen_def  = OpaqueDefinition.new(opaque_type,
                                                 intermediate_type, options, convert_code_generator) 
                orogen_def.caller = caller
                @opaques << orogen_def
                @opaques = opaques.
                    sort_by { |orogen_def| orogen_def.type.name }
                opaque_type
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

            # Handle a load that points to a file in this typekit's source tree
            def handle_local_load(file)
                rel = Pathname.new(file).relative_path_from(Pathname.new(base_dir))
                return file if rel.each_filename.first == ".."

                local_type_dir = Pathname.new(includes_dir)
                rel_to_type_dir = Pathname.new(file).relative_path_from(local_type_dir)
                if rel_to_type_dir.each_filename.first == ".."
                    # If the type is within a subdirectory called as the
                    # typekit, remove the duplicate
                    elements = rel.each_filename.to_a
                    if elements.first != self.name
                        elements.unshift self.name
                    end
                    target = File.join(includes_dir, *elements)
                    Generation.create_or_update_symlink(file, target)
                    target
                else file
                end
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
            #
            # @raises LoadError if the file does not exist
	    def load(file, add = true, user_options = Hash.new)
                if !user_options.respond_to?(:to_hash) 
                    raise ArgumentError, "expected an option has as third argument, got #{user_options.inspect}"
                end

                if match = /(\w+)\/Types\.hpp$/.match(file)
                    project_name = match[1]
                    if project.has_typekit?(project_name) || project.name == project_name
                        raise ArgumentError, "cannot use a header called #{file} as it clashes with the Types.hpp header generated by orogen for the #{project_name} project"
                    end
                end

                include_dirs = self.include_dirs

                # Get the full path for +file+
                file =
                    if File.exists?(file) # Local file
                        File.expand_path(file)
                    else # File from used libraries/task libraries
                        dir = include_dirs.find { |dir| File.exists?(File.join(dir, file)) }
                        if !dir
                            raise LoadError, "cannot find #{file} in #{include_dirs.to_a.join(":")}"
                        end
                        loaded_files_dirs << dir
                        File.join(dir, file)
                    end

                # If it is a local header, symlink it to the "right" place in
                # typekit/types and load that
                file = handle_local_load(file)

                # And resolve it to an include statement
                include_path = include_dirs.map { |d| Pathname.new(d) }
                inc = resolve_full_include_path_to_relative(file, include_path)
                included_files << inc

                this_options = [add, user_options]
                if pending_load_options != this_options
                    perform_pending_loads
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
                        RTT_CPP.warn "ignoring #{type.name} as multi-dimensional arrays cannot be represented in CORBA IDL"
                        to_delete << type

                    elsif type < Typelib::PointerType
                        RTT_CPP.warn "ignoring #{type.name} as pointers are not allowed"
                        to_delete << type

                    elsif type.name == "/std/vector</bool>"
                        RTT_CPP.warn "std::vector<bool> is unsupported in oroGen due to its special nature. Use std::vector<uint8_t> instead."
                        to_delete << type

                    elsif type < Typelib::CompoundType
                        type.each_field do |field_name, _|
                            if field_name !~ /^[a-zA-Z]/
                                RTT_CPP.warn "ignoring #{type.name} as its field #{field_name} does not start with an alphabetic character, which is forbidden in CORBA IDL"
                                to_delete << type
                                break
                            end
                        end
                    end
                end

                to_delete.each do |type|
                    deleted_types = registry.remove(type)
                    deleted_types.each do |dep_type|
                        next if to_delete.include?(dep_type)
                        RTT_CPP.warn "ignoring #{dep_type.name} as it depends on #{type.name} which is ignored"
                    end
                end
            end

            # Resolves a full path into the one that should be used when taking
            # into account the include path
            #
            # @param [String] file the full path to the include file
            # @param [Array<Pathname>] include_path the set of include dirs
            # @return [String]
            def resolve_full_include_path_to_relative(file, include_path)
                file = Pathname.new(file)
                # Find the base include
                include_candidates = include_path.map do |inc|
                    rel = file.relative_path_from(inc).cleanpath.to_path
                    if rel !~ /^\.\.\//
                        rel
                    end
                end
                include_candidates.compact.min_by { |inc| inc.size }
            end

            # Returns an existing orogen_include metadata entry for the given
            # type, or nil if none exists so far
            #
            # @param type [Model<Type>] a type model
            # @return [Array,nil] either a non-empty set of entries for
            #   orogen_include or nil if none could be found
            def existing_orogen_include_for_type(type)
                if imported_types.include?(type.name)
                    metadata = imported_types.get(type.name).metadata
                    if metadata.include?('orogen_include')
                        return metadata.get('orogen_include')
                    end
                end
                if self.registry.include?(type.name)
                    metadata = self.registry.get(type.name).metadata
                    if metadata.include?('orogen_include')
                        return metadata.get('orogen_include')
                    end
                end
                metadata = type.metadata
                if metadata.include?('orogen_include')
                    return metadata.get('orogen_include')
                end
                nil
            end

            def orogen_include_of_type(type, file_to_include)
                # 'Types with deference' are vectors and arrays. Arrays
                # are ignored. Vectors are hardcoded to :vector
                if type.respond_to?(:deference)
                    deference_includes = orogen_include_of_type(type.deference, file_to_include)
                    # if the type used in the opaque/ro_ptr/container has no
                    # include, the container doesn't need one as well?
                    if !deference_includes
                        return
                    end

                    container_includes = []
                    if type <= Typelib::ContainerType
                        if type.name == '/std/string'
                            container_includes = [':string']
                        elsif type.container_kind == '/std/vector'
                            container_includes = [':vector']
                        else
                            raise ArgumentError, "unexpected container type #{type.container_kind}"
                        end
                    end
                    return container_includes + deference_includes
                elsif type <= Typelib::NullType
                    return []
                elsif type <= Typelib::NumericType
                    if type.integer? then return [':boost/cstdint.hpp']
                    else return []
                    end
                elsif existing = existing_orogen_include_for_type(type)
                    return existing
                else
                    if !(location = type.metadata.get('source_file_line').first)
                        return
                    end

                    file, line = location.split(':')
                    if !File.file?(file)
                        RTT_CPP.debug("resolve_registry_includes: deleting non-existing 'line' entry in metadata 'source_file_line'=#{location}")
                        type.metadata.delete('source_file_line')
                        return
                    end

                    if orogen_include = file_to_include[file][Integer(line)]
                        return ["#{self.name}:#{orogen_include}"]
                    else raise ArgumentError, "no entry for '#{file}:#{line}' in the provided file-to-include mapping of typekit '#{self.name}'"
                    end
                end
            end
            
            def compute_orogen_include_on_type(type, file_to_include)
                if includes = orogen_include_of_type(type, file_to_include)
                    type.metadata.set('orogen_include', *includes)
                    includes
                elsif has_pending_loads?
                    perform_pending_loads
                    compute_orogen_include_on_type(type, file_to_include)
                else raise ArgumentError, "cannot compute include information for #{type.name}. If it is an opaque, you must either load the header which defines it with import_types_from, or specify the relevant information with the orogen_include option"
                end
            end

            # Resolves the orogen_include metadata for each type in the given
            # registry
            #
            # It computes the set of #include <> statements that are required to
            # get access on the C++ side to each type in the registry. It is
            # saved in the orogen_include metadata information of the types.
            #
            # @param [Typelib::Registry] registry whose types should be
            #   resolved
            # @param [{String=>Array<String>}] file_to_include mapping from a
            #   source file and line to the toplevel include that relates to
            #   this file
            def resolve_registry_includes(registry, file_to_include)
                queue = Array.new
                registry.each do |type|
                    _, template_args = Typelib::GCCXMLLoader.parse_template(type.name)
                    template_args = template_args.map do |type_name|
                        if registry.include?(type_name)
                            registry.get(type_name)
                        end
                    end.compact
                    queue << [type, template_args]
                end
                queue = queue.sort_by { |type, template_args| [template_args.size, type.name.size] }

                while !queue.empty?
                    # If this is a template, we need to add the relevant
                    # includes for the parameters as well. We need to do some
                    # form of ordering for that to work ...
                    queue.delete_if do |type, template_args|
                        has_unresolved_args = template_args.any? do |template_arg_type|
                            queue.include?(template_arg_type)
                        end
                        if !has_unresolved_args
                            compute_orogen_include_on_type(type, file_to_include)
                            true
                        end
                    end
                end
            end

            def resolve_toplevel_include_mapping(toplevel_files, options)
                includes = options[:include].map { |v| "-I#{v}" }
                defines  = options[:define].map { |v| "-D#{v}" }

                preprocessed = Tempfile.open('orogen_gccxml_input') do |io|
                    toplevel_files.each do |path|
                        io.puts "#include <#{path}>"
                    end
                    io.flush
                    arch_type = "-m64"
                    if 1.size == 4
                      arch_type = "-m32"
                    end
                    result = IO.popen(["gccxml", "--preprocess", arch_type, *includes, *defines, io.path]) do |io|
                        io.read
                    end
                    if !$?.success?
                        raise ArgumentError, "failed to preprocess #{toplevel_files.join(" ")}"
                    end
                    result
                end

                owners = Hash.new { |h,k| h[k] = Array.new }
                current_file = [[]]
                preprocessed.each_line do |line|
                    if line =~ /# (\d+) "(.*)"(?: (\d))?/
                        lineno, file, mode = Integer($1), $2, $3

                        if mode == "1"
                            toplevel_file =
                                if toplevel_files.include?(file) then file
                                else current_file.last[0]
                                end
                            current_file.push [toplevel_file, file, lineno]
                        elsif mode == "2"
                            current_file.pop
                        end
                        current_file.last[2] = lineno
                    else
                        toplevel, file, lineno = *current_file.last
                        owners[file][lineno] = toplevel
                        current_file.last[2] += 1
                    end
                end
                return preprocessed, owners
            end

            def make_load_options(pending_loads, user_options)
                options = { :opaques_ignore => true, :merge => false, :required_files => pending_loads.to_a }
                # GCCXML can't parse vectorized code, and the Typelib internal
                # parser can't parse eigen at all. It is therefore safe to do it
                # here
                options[:define] = ['EIGEN_DONT_VECTORIZE', "OROCOS_TARGET=#{RTT_CPP.orocos_target}", '__orogen2']

                options[:include] = self.include_dirs.dup
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

                opaque_names = opaques.map { |opdef| opdef.type.name }
                options[:opaques] = opaque_names
                Kernel.filter_options options, :include, :define
            end

            def has_pending_loads?
                !pending_loads.empty?
            end

            def perform_pending_loads
                return if pending_loads.empty?
                loads = pending_loads.dup
                pending_loads.clear

                add, user_options = *pending_load_options

                include_dirs = self.include_dirs.to_a
                if automatic_dir
                    include_dirs << File.join(automatic_dir, "types")
                end

                file_registry = Typelib::Registry.new
                file_registry.merge opaque_registry

                preprocess_options, options = make_load_options(loads, user_options)
                preprocessed, include_mappings = resolve_toplevel_include_mapping(loads, preprocess_options)

                include_path = include_dirs.map { |d| Pathname.new(d) }
                pending_loads_to_relative = loads.inject(Hash.new) do |map, path|
                    map[path] = resolve_full_include_path_to_relative(path, include_path)
                    map
                end

                include_mappings.each do |file, lines|
                    lines.map! { |inc| pending_loads_to_relative[inc] }
                end

                Tempfile.open("orogen_pending_loads") do |io|
                    io.write preprocessed
                    io.flush

                    begin
                        file_registry.import(io.path, 'c', options)
                        filter_unsupported_types(file_registry)
                        resolve_registry_includes(file_registry, include_mappings)
                        registry.merge(file_registry)
                        if project
                            project.registry.merge(file_registry)
                        end
                    rescue Exception => e
                        raise ArgumentError, "cannot load one of the header files #{loads.join(", ")}: #{e.message}", e.backtrace
                    end
                end

                if add
                    self.loads.concat(loads.to_a)
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
            BASE_TYPES_NEEDED_TRANSPORTS = %w{typelib ros}

            def normalize_registry
                base = self.registry.dup

                # Properly define the headers we want to use for cstdint headers
                [1, 2, 4, 8].each do |int_size|
                    if base.include?("/int#{int_size * 8}_t")
                        base.get("/int#{int_size * 8}_t").
                            metadata.set("orogen_include", "boost/cstdint.hpp")
                    end
                    if base.include?("/uint#{int_size * 8}_t")
                        base.get("/uint#{int_size * 8}_t").
                            metadata.set("orogen_include", "boost/cstdint.hpp")
                    end
                end

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

                result
            end

            def imported_array_of?(type)
                typename = if type.respond_to?(:name) then type.name
                           else type.to_str
                           end

                imported_typelist.any? { |str| str =~ /#{Regexp.quote(typename)}(\[\d+\])+/ }
            end

            # Computes the set of typekits that are required to get the given
            # types.
            #
            # self is never included in the result
            def typekits_required_for(types)
                result = Set.new
                types.each do |type|
                    loop do
                        imported_typekits.each do |tk|
                            if type < Typelib::ArrayType 
                                if tk.defines_array_of?(type.deference)
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
                result
            end

            # List of typekits that this typekit depends on
            def used_typekits
                @registry = normalize_registry
                typekits_required_for(registry.each)
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
                        if deps = plg.dependencies
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
                    [name.gsub('/', '::').gsub('<::', '< ::').gsub('>>','> >')]
                end
            end


            def m_types_code
                @m_types_code
            end

            def m_type_exists?(type)
                target_typename = intermediate_type_name_for(type)
                current_def = begin
                                  find_type(target_typename)
                              rescue Typelib::NotFound
                              end

                return false if !current_def

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
                                if intermediate_type_for(expected[1]) != current[1]
                                    raise "#{current_def.name} is already defined, but does not match the expected definition. Did you define it yourself ?"
                                end
                            end

                            type_fields.pop
                            current_fields.pop
                        end
                    end
                end
                true
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
                needed_definitions = self_types.
                    find_all { |t| t.contains_opaques? && !t.opaque? }

                # Sort the types by the order in which we should generate their
                # intermediates. This is needed when one m-type requires another
                # m-type that we generate in this typekit as well
                to_generate = []
                while !needed_definitions.empty?
                    needed_definitions.delete_if do |type|
                        next(true) if m_type_exists?(type)
                        next if type.dependencies.any? { |t| needed_definitions.include?(t) }
                        to_generate << type
                    end
                end

                options = { :include => include_dirs.dup }

                to_generate.each do |type|
                    needed_type_definitions = type.direct_dependencies.map do |needed_type|
                        find_type(intermediate_type_for(needed_type), true)
                    end
                    typekit = self
                    marshalling_code = Generation.
                        render_template 'typekit', 'marshalling_types.hpp', binding

                    path = Generation.save_automatic 'typekit', 'types', self.name, "m_types", "#{type.method_name(true)}.hpp", marshalling_code
                    self.load(path, true, options)
                end
                true
            end

            def do_import(registry, path, kind, options)
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
			OroGen::Gen::RTT_CPP.info "you will not be able to marshal #{type.name} as XML, it contains 64bit integers"
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

            def save_automatic_public_header(*args)
                rel = File.join(self.name, 'typekit', *args[0..-2])
                Generation.save_generated(true, automatic_dir, INCLUDE_DIR_NAME, rel, args[-1])
                rel
            end

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

                slice_size = RTT_CPP.typekit_slice
                while slice_size > 1 && code_snippets.size / slice_size < RTT_CPP.typekit_slice_minimum
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

                # Generate opaque-related stuff first, so that we see them in
                # the rest of the typelib-registry-manipulation code
                handle_opaques_generation(registry)
                perform_pending_loads

		# Do some registry mumbo-jumbo to remove unneeded types to the
                # dumped registry
                @registry = normalize_registry
		minimal_registry = @registry.dup
                generated_types = self_types.to_value_set

                self_opaques = self.self_opaques.sort_by { |opdef| opdef.type.name }

		issue_warnings(generated_types, minimal_registry)
                generate_typedefs(generated_types, minimal_registry)

                # The first array is the set of types for which convertion
                # functions are generated. The second is the set of types that
                # are actually registered into the RTT type system
                #
                # The two arrays are sorted so that we don't have to recompile
                # unnecessarily (the original sets are hashes, and therefore don't
                # have a stable order).
                converted_types = generated_types.
                    find_all { |type| !(type <= Typelib::NumericType) }.
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
                        generated_types.find_all do |type|
                            !m_type?(type) && !(type <= Typelib::NumericType)
                        end.to_value_set

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

                # Save all the types that this specific typekit handles,
                # including aliases
                registered_typenames = registered_types.map(&:name).to_set
                all_names = Hash.new
                minimal_registry.each(:with_aliases => true) do |name, type|
                    all_names[type.name] ||= []
                    all_names[type.name] << name
                end
                typelist_txt = []
                generated_types.each do |type|
                    is_exported = registered_typenames.include?(type.name) ? '1' : '0'
                    metadata = minimal_registry.get(type.name).metadata
                    metadata.add 'orogen_defining_typekits', self.name
                    if is_exported
                        metadata.add 'orogen_exporting_typekits', self.name
                    end
                    all_names[type.name].each do |type_name|
                        typelist_txt << "#{type_name} #{is_exported}"
                    end
                end
                save_automatic "#{name}.typelist",
                    typelist_txt.sort.join("\n")

                # Generate the XML representation of the generated type library,
                # and add opaque information to it
                plain_registry = minimal_registry.to_xml
                doc = REXML::Document.new(plain_registry)
                doc.each_element('//opaque') do |opaque_entry|
                    spec = opaque_specification(opaque_entry.attributes['name'])
                    opaque_entry.add_attributes(
                        'marshal_as' => spec.intermediate,
                        'includes' => spec.includes.join(':'),
                        'needs_copy' => (spec.needs_copy? ? '1' : '0'))
                end

                modified_tlb = String.new
                doc.write(modified_tlb)
                save_automatic "#{name}.tlb", modified_tlb

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

                public_header_files, plugin_header_files, implementation_files = [], [], []

                # For backward compatibility
                if File.symlink?(old_symlink = File.join(automatic_dir, 'Opaques.hpp'))
                    FileUtils.rm_f old_symlink
                end

		type_header = Generation.render_template('typekit/Types.hpp', binding)
		public_header_files << save_automatic_public_header("Types.hpp", type_header)
		type_header = Generation.render_template('typekit/TypesDeprecated.hpp', binding)
		save_automatic("Types.hpp", type_header)
		type_header = Generation.render_template('typekit/OpaquesDeprecated.hpp', binding)
		save_automatic("Opaques.hpp", type_header)
                boost_serialization = Generation.render_template "typekit/BoostSerialization.hpp", binding
                public_header_files << save_automatic_public_header("BoostSerialization.hpp", boost_serialization)
                tk_hpp = Generation.render_template "typekit/Plugin.hpp", binding
		public_header_files << save_automatic_public_header("Plugin.hpp", tk_hpp)
                tk_cpp = Generation.render_template "typekit/Plugin.cpp", binding
		implementation_files << save_automatic("Plugin.cpp", tk_cpp)

                # Generate the opaque convertion files
                if has_opaques?
                    intermediates_hpp = Generation.render_template 'typekit/OpaqueConvertions.hpp', binding
                    public_header_files <<
                        save_automatic_public_header("OpaqueConvertions.hpp", intermediates_hpp)

                    intermediates_cpp = Generation.render_template 'typekit/OpaqueConvertions.cpp', binding
                    implementation_files <<
                        save_automatic("OpaqueConvertions.cpp", intermediates_cpp)

                    fwd_hpp = Generation.render_template 'typekit/OpaqueFwd.hpp', binding
                    public_header_files <<
                        save_automatic_public_header("OpaqueFwd.hpp", fwd_hpp)

                    types_hpp = Generation.render_template 'typekit/OpaqueTypes.hpp', binding
                    public_header_files <<
                        save_automatic_public_header("OpaqueTypes.hpp", types_hpp)

                    if has_opaques_with_templates?
                        user_hh = Generation.render_template 'typekit/Opaques.hpp', binding
                        user_cc = Generation.render_template 'typekit/Opaques.cpp', binding
                        save_user("Opaques.hpp", user_hh)
                        implementation_files <<
                            save_user("Opaques.cpp", user_cc)

                        Generation.create_or_update_symlink(
                            File.join(user_dir, "Opaques.hpp"),
                            File.join(automatic_dir, INCLUDE_DIR_NAME, self.name, 'typekit', "Opaques.hpp"))
                    end
                end

                each_plugin do |plg|
                    plg_typesets = type_sets.dup
                    headers, impl = plg.generate(plg_typesets)
                    plugin_header_files.concat(headers)
                    implementation_files.concat(impl)
                end

                if standalone?
                    FileUtils.mkdir_p File.join(automatic_dir, 'config')
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
                cmake_build = Generation.render_template 'typekit', 'build.cmake', binding
                save_automatic("build.cmake", cmake_build)
                manifest = Generation.render_template 'typekit', 'manifest.xml', binding
                save_automatic("manifest.xml", manifest)
                #package = Generation.render_template 'typekit', 'package.xml', binding
                #save_automatic("package.xml", package)
                makefile = Generation.render_template 'typekit', 'Makefile', binding
                save_automatic("Makefile", makefile)

                # Finished, create the timestamp file
                Generation.cleanup_dir(automatic_dir)
                Generation.touch File.join(automatic_dir, 'stamp')
	    end

            def cmake_relative_path(file, *subdir)
                "#{Pathname.new(file).relative_path_from(Pathname.new(File.join(automatic_dir, *subdir)))}"
            end

            def type_info_includes_for_type(type)
                if type.opaque?
                    return type_info_includes_for_type(intermediate_type_for(type))
                elsif TypekitMarshallers::TypeInfo::Plugin.rtt_scripting?
                   result = ["#{self.name}/typekit/BoostSerialization.hpp", type.info_type_header]
                   if type.full_name == "/std/string"
                       result << "rtt/typekit/StdStringTypeInfo.hpp"
                   end
                   result
                else
                   ["rtt/types/PrimitiveTypeInfo.hpp", "rtt/types/TemplateConnFactory.hpp"]
                end
            end

            # Returns the set of includes that should be added to get access to
            # the given type
            #
            # @param [Model<Typelib::Type>] type the type subclass
            # @return [Array<String>]
            # @see cxx_gen_includes
            def include_for_type(type)
                if includes = existing_orogen_include_for_type(type)
                    return includes.map { |s| s.split(':').last }
                end

                if type.opaque?
                    raise ConfigError, "no includes known for #{type.name}, This is an opaque, and you must either call import_types_from on a header that defines it, or provide the :include option to the opaque definition"
                else
                    raise InternalError, "no includes known for #{type.name} defined in #{type.metadata.get("source_file_line")}"
                end
            end


            # Given a set of includes, returns the code that includes them in a
            # C++ file
            #
            # @param [String] includes
            # @return [String]
            def cxx_gen_includes(*includes)
                includes.to_set.map do |inc|
                    "#include <#{inc}>"
                end.sort.join("\n") + "\n"
            end
            def relative_path(file, *subdir)
                "#{Pathname.new(file).relative_path_from(Pathname.new(File.join(automatic_dir, *subdir)))}"
            end
	end
    end
    end
end

