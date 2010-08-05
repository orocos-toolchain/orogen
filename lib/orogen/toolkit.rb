require 'typelib'
require 'tempfile'
require 'find'
require 'orogen/base'
require 'utilrb/kernel/options'
require 'nokogiri'

require 'orogen/marshallers/corba'
require 'orogen/marshallers/iostream'
require 'orogen/marshallers/property_bag'

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

        def self.arg_type; "#{cxx_name} const&" end
        def self.ref_type; "#{cxx_name}&" end

        def self.method_name(fullname = true)
            base = if fullname then full_name('_', true)
                   else basename('_')
                   end
            base.gsub(/[<>\[\] ]/, '_')
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

        def self.inlines_code?; false end
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

	def self.contains_int64?; size == 8 && integer?  end

        def self.inlines_code?; superclass.eql?(NumericType) end
    end

    class ContainerType
	def self.contains_int64?; deference.contains_int64?  end
        def self.contains_opaques?; deference.contains_opaques? end

        def self.cxx_name
            kind = container_kind.
                gsub('/', '::').
                gsub(/^::/, '')
            if name =~ /</
                normalize_cxxname(kind + "< " + deference.cxx_name + " >")
            else
                normalize_cxxname(kind)
            end
        end

        def self.code_copy(toolkit, result, indent, dest, src, method)
            collection_name, element_type = container_kind, deference.name
            element_type = registry.build(element_type)

            if (container_kind == "vector")
                result << "#{indent}#{dest}.reserve(#{src}.size());\n"
            end

            result << <<-EOT
#{indent}for(#{cxx_name}::const_iterator it = value.begin(); it != value.end(); ++it)
#{indent}{
            EOT

            if yield(element_type)
                result << "#{indent}    #{dest}.push_back(*it);\n"
            else
                result << "#{indent}    #{dest}.push_back(<%= cxx_name %>::value_type());\n"
                result << "#{indent}    #{method}(*#{dest}.rbegin(), *it);\n";
            end

            result << "#{indent}}\n";
        end

        def self.to_intermediate(toolkit, result, indent)
            code_copy(toolkit, result, indent, "intermediate", "value", "to_intermediate") do |type|
                !type.contains_opaques?
            end
        end
        def self.from_intermediate(toolkit, result, indent)
            code_copy(toolkit, result, indent, "value", "intermediate", "from_intermediate") do |type|
                !type.contains_opaques?
            end
        end
    end

    class EnumType
        def self.to_string(toolkit, result, indent)
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
	def self.contains_int64?; enum_for(:each_field).any? { |_, type| type.contains_int64? } end
	def self.contains_opaques?; enum_for(:each_field).any? { |_, type| type.contains_opaques? } end

        def self.code_copy(toolkit, result, indent, dest, src, method, error_handling)
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
                        s_array = if src == "value" then s_src
                                  else s_dest
                                  end
                        s_array << ", #{field_type.length}"
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

        def self.to_intermediate(toolkit, result, indent)
            code_copy(toolkit, result, indent, "intermediate", "value", "to_intermediate", false) do |field_name, field_type|
                !field_type.contains_opaques?
            end
        end
        def self.from_intermediate(toolkit, result, indent)
            code_copy(toolkit, result, indent, "value", "intermediate", "from_intermediate", false) do |field_name, field_type|
                !field_type.contains_opaques?
            end
        end
    end

    class ArrayType
	def self.contains_int64?; deference.contains_int64?  end
        def self.contains_opaques?; deference.contains_opaques? end

        def self.arg_type; "#{deference.cxx_name} const*" end
        def self.ref_type; "#{deference.cxx_name}*" end

        def self.code_copy(toolkit, result, indent, dest, src, method)
            element_type = registry.build(deference.name)

            allocate_index do |i|
                result << <<-EOT
#{indent}for(int #{i} = 0; #{i} < length; ++#{i})
#{indent}{
                EOT

                if string = yield(element_type, "#{dest}[#{i}]", "#{src}[#{i}")
                    if !string.respond_to?(:to_str)
                        result << "#{indent}  #{dest}[#{i}] = #{src}[#{i}];\n"
                    else
                        result << string
                    end
                else
                    s_src  = "#{src}[#{i}]"
                    s_dest = "#{dest}[#{i}]"
                    if element_type < ArrayType
                        s_array = if src == "value" then s_src
                                  else s_dest
                                  end
                        s_array << ", #{element_type.length}"
                    end

                    result << "#{indent}    if (!#{method}(#{s_dest}, #{s_src})) return false;\n";
                end

                result << "#{indent}}\n";
            end
	    result
        end

        def self.to_intermediate(toolkit, result, indent)
            code_copy(toolkit, result, indent, "intermediate", "value", "to_intermediate") do |type|
                !type.contains_opaques?
            end
        end

        def self.from_intermediate(toolkit, result, indent)
            code_copy(toolkit, result, indent, "value", "intermediate", "from_intermediate") do |type|
                !type.contains_opaques?
            end
        end
    end

    class Registry
        # Returns true if +type+ is handled by the toolkit that is included in
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
        # Data structure that represents the definition for an opaque type
        #
        # See Toolkit#opaque_type
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

	class Toolkit
            # The component this toolkit is part of
	    attr_reader :component
            # The set of headers loaded by #load, as an array of absolute paths
            attr_reader :loads

            # The set of external headers that have been imported in this
            # toolkit. I.e. it contains the headers that are loaded from a
            # depended-upon library for instance.
            def external_loads
                loads.find_all do |name|
                    name !~ /#{Regexp.quote(component.base_dir)}/
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
            # EXPERIMENTAL
            #
            # Change or read the current type export policy. This policy drives
            # what types orogen will import in the RTT type system.
            #
            # If the :all policy is used (the default), then all types that have
            # been imported through header files will be included. This can
            # generate a lot of code, and therefore produce high compilation
            # times and big toolkit libraries.
            #
            # If the :used policy is used, then will only be exported the types
            # that are actually used in a task context of the toolkit's project.
            # This is most of the time a good idea.
            #
            # If the :selected policy is used, then only types that have been
            # explicitely selected with #export_types will be included.
            dsl_attribute :type_export_policy do |new_policy|
                new_policy = new_policy.to_sym
                if !TYPE_EXPORT_POLICIES.include?(new_policy)
                    raise ArgumentError, "invalid type export policy #{new_policy.inspect}, allowed are: :#{TYPE_EXPORT_POLICIES.join(", :")}"
                end
                new_policy
            end

            # Select a set of types to be exported through the RTT type system,
            # instead of exporting everything. This is meant to reduce the
            # toolkit's code size and compilation times tremendously.
            #
            # EXPERIMENTAL
            #
            # See also #type_export_policy
            def export_types(*selection)
                type_export_policy :selected
                @selected_types.concat(selection.map { |name| find_type(name) })
            end

            attr_reader :selected_types

	    attr_reader :registry
            attr_reader :preloaded_registry
            attr_reader :opaques
            attr_reader :opaque_registry
            attr_reader :pending_load_options
            attr_reader :pending_loads

	    dsl_attribute :blob_threshold do |value|
		value = Integer(value)
		if value == 0; nil
		else value
		end
	    end

            # Returns the Typelib::Type subclass that represents the type whose
            # name is given. If the type is a derived type (pointer, array or
            # container), then it will be built on the fly.
	    def find_type(type)
                if type.respond_to?(:to_str)
                    type_name = type.gsub('::', '/')
                    type_name = Typelib::Type.normalize_typename(type_name)
                    begin
                        registry.build(type_name)
                    rescue Typelib::NotFound
                        if type_name =~ /^([^<]+)<(.*)>$/
                            container_name = $1
                            element_name   = $2
                            element_type   = find_type(element_name)
                            if !element_type
                                raise ArgumentError, "the type #{element_name.inspect} is not defined"
                            elsif element_type.contains_opaques?
                                raise ArgumentError, "cannot create a container of #{element_name}: it contains an opaque type or is opaque itself"
                            end
                            component.registry.define_container(container_name,
                                            component.registry.build(element_name))
                            registry.define_container(container_name, element_type)
                        elsif type = component.registry.build(type_name)
                            while type.respond_to?(:deference)
                                type = type.deference
                            end

                            type_def = component.registry.minimal(type.name)
                            registry.merge(type_def)
                            registry.build(type_name)
                        end
                    end
                elsif type.kind_of?(Class) && type <= Typelib::Type
                    type
                else
                    raise ArgumentError, "expected a type object, got #{type}"
                end

            rescue Typelib::NotFound => e
                if !pending_loads.empty?
                    perform_pending_loads
                    retry
                end
                raise ArgumentError, e.message, e.backtrace
	    end

            # True if we are generating for Linux
            def linux?;     component.linux? end
            # True if we are generating for Xenomai
            def xenomai?;   component.xenomai? end

	    # Set of directories in which the header files that have been
	    # loaded lie. It is used to generate the Cflags: line in the
	    # pkg-config file
	    attr_reader :loaded_files_dirs

	    def initialize(component)
		@component = component

                @internal_dependencies = []
		@corba_enabled      = nil
		@imports, @loads    = [], []
		@registry           = Typelib::Registry.new
		@preloaded_registry = Typelib::Registry.new
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
                @selected_types = Array.new

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
                    Generation.render_template('toolkit/smart_ptr.cpp', binding)
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
                @opaques = opaques.
                    sort_by { |orogen_def| orogen_def.type.name }
            end

            # True if there are some opaques in this toolkit
            def has_opaques?
                !opaques.empty?
            end

            # True if some opaques require to generate templates
            def has_opaques_with_templates?
                opaques.any? { |op| op.generate_templates? }
            end

            # Returns the include directories that should be used for file
            # loading
            def include_dirs
                include_dirs = []
                include_dirs << component.base_dir if component.base_dir
                include_dirs.concat(component.used_toolkits.map   { |tk| tk.include_dirs }.flatten)
                include_dirs.concat(component.used_libraries.map  { |pkg| pkg.include_dirs }.flatten)
                include_dirs.concat(component.used_task_libraries.map { |component| component.include_dirs }.flatten)
                include_dirs
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
	    def load(file, preload = false, add = true, user_options = Hash.new)
                this_options = [preload, add, user_options]
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
                            raise ArgumentError, "cannot find #{file} in #{include_dirs.join(":")}"
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

                preload, add, user_options = *pending_load_options

                include_dirs = self.include_dirs

                file_registry = Typelib::Registry.new
                file_registry.merge opaque_registry

                options = { :opaques_ignore => true, :merge => false, :required_files => pending_loads.to_a }
                # GCCXML can't parse vectorized code, and the Typelib internal
                # parser can't parse eigen at all. It is therefore safe to do it
                # here
                options[:define] = ['EIGEN_DONT_VECTORIZE', '__orogen']
                options[:include] = include_dirs.dup
                options = options.merge(user_options) do |a, b|
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

                File.open("orogen_pending_loads", 'w') do |io|
                    pending_loads.each do |f|
                        io.puts "#include \"#{f}\""
                    end
                    io.flush

                    begin
                        file_registry.import(io.path, 'c', options)
                        filter_unsupported_types(file_registry)
                        registry.merge(file_registry)
                        preloaded_registry.merge(file_registry) if preload
                        component.registry.merge(file_registry)
                    rescue RuntimeError => e
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
                list = loads.find_all { |path| path =~ /^#{Regexp.quote(component.base_dir)}\// }
                if absolute
                    list
                else
                    list.map do |p|
                        relative = p.gsub(/^#{Regexp.quote(component.base_dir)}\//, '')
                        dest = relative.
                            gsub(/^#{Regexp.quote(Generation::AUTOMATIC_AREA_NAME)}\//, '').
                            gsub(/^#{component.name}\//, '')

                        [relative, dest]
                    end
                end
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

            def normalize_registry
		minimal_registry = self.registry.
		    minimal(preloaded_registry).
		    minimal(component.rtt_registry)
		minimal_registry = component.used_toolkits.
		    inject(minimal_registry) { |reg, tk| reg.minimal(tk.registry) }

                @registry = minimal_registry
            end

            # List of toolkits that this toolkit depends on
            def used_toolkits
                normalize_registry
                result = Set.new

                # We depend on the toolkits that define types that are used in
                # our types (ouch), as they define the convertion functions that
                # our toolkit will use.
                #
                # We therefore must not use self_types there but really
                # registry.each_type. Note that the registry is minimal, i.e.
                # contains only our own types plus the types that are needed to
                # define them, and this is therefore not a problem.
		registry.each_type(true) do |name, type|
                    loop do
                        component.used_toolkits.each do |tk|
                            if type < Typelib::ArrayType 
                                if tk.has_array_of?(type.deference)
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

            # Returns the set of pkg-config packages this toolkit depends on
            def dependencies
                result = []

                # We must link to libraries in case the types we are getting
                # from them has constructors/destructors that the library
                # provides.
                component.used_libraries.each do |pkg|
                    needs_link = component.toolkit_libraries.include?(pkg)
                    result << BuildDependency.new(pkg.name.upcase, pkg.name, false, true, needs_link)
                end

                # We must link to toolkits that define our types, as we are
                # going to reuse the convertion functions that they define
                used_toolkits.each do |tk|
                    result << BuildDependency.new(tk.name.upcase + "_TOOLKIT", tk.pkg_name, false, true, true)
                end

                # We must include the toolkits that define types that are used
                # in the other toolkits types

                if corba_enabled?
                    used_toolkits.each do |tk|
                        result << BuildDependency.new(tk.name.upcase + "_TRANSPORT_CORBA", tk.pkg_corba_name, true, true, true)
                    end
                end

                result.to_a.sort_by { |dep| dep.var_name }
            end

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

            def handle_opaques_generation(generated_types)
                # Make sure all opaque intermediate types are existing or can be
                # instanciated
                opaques.each do |opaque_def|
                    find_type(opaque_def.intermediate)
                end

                # Generate some type definitions for the pocosim marshalling. In
                # practice, we generate C code that we merge back into the
                # repository
                generate_all_marshalling_types = false
                generated_types = registry.enum_for(:each_type).
                    find_all { |t| !component.imported_type?(t.name) && !t.inlines_code? }
		toolkit = self
                catch(:nothing_to_define) do
                    Tempfile.open('orogen') do |io|
                        marshalling_code = Generation.render_template 'toolkit/marshalling_types.hpp', binding
                        io << marshalling_code
                        io.flush

                        registry.import(io.path, 'c', :merge => false)
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

            # Returns the set of type names defined in this toolkit. This is
            # different from self_types, as it returns a set of type names (i.e.
            # strings), and also because it includes the aliases defined by the
            # toolkit.
            def self_typenames
		generated_types = []
		registry.each_type(true) do |name, type|
                    next if component.imported_type?(name)
		    if !type.inlines_code?
			generated_types << name
		    end
		end
                generated_types
            end

            # Returns the set of types that are specifically handled by this
            # toolkit
            def self_types
		generated_types = []
		registry.each_type do |type|
                    next if component.imported_type?(type.name)
                    generated_types << type
		end
                generated_types
            end

            # Helper method that returns the code needed to get an
            # +intermediate+ variable of the right type, containing the data in
            # +value+.
            def code_toIntermediate(intermediate_type, needs_copy, indent)
                if needs_copy
                    "#{indent}#{intermediate_type.cxx_name} intermediate;\n" +
                    "#{indent}#{component.name}::to_intermediate(intermediate, value);\n"
                else
                    "#{intermediate_type.cxx_name} const& intermediate = #{component.name}::to_intermediate(value);"
                end
            end

            # Helper method that returns the code needed to update an opaque
            # type based on the data from an intermediate variable
            def code_fromIntermediate(intermediate_type, needs_copy, indent)
                if needs_copy
                    "#{indent}#{component.name}::from_intermediate(value, intermediate);\n"
                else
                    "#{indent}if (#{component.name}::from_intermediate(value, intermediate.get()))\n" +
                    "#{indent}    intermediate.release();\n"
                end
            end

            attr_reader :template_instanciation_files

	    def generate
		toolkit = self

                # Generate the state enumeration types for each of the task
                # contexts, and load it
                if component.self_tasks.any?(&:extended_state_support?)
                    state_types = Generation.render_template "tasks", "TaskStates.hpp", binding
                    Generation.save_automatic "#{component.name}TaskStates.hpp", state_types
                    load File.join(AUTOMATIC_AREA_NAME, "#{component.name}TaskStates.hpp"), false
                end

                # Load any queued file
                perform_pending_loads

                FileUtils.mkdir_p File.join(component.base_dir, AUTOMATIC_AREA_NAME, 'toolkit')

                # Populate a fake installation directory so that the include
                # files can be referred to as <project_name>/header.h
                fake_install_dir = File.join(component.base_dir, AUTOMATIC_AREA_NAME, component.name)
                FileUtils.mkdir_p fake_install_dir

                self.local_headers(false).each do |path, dest_path|
                    dest_path = File.join(fake_install_dir, dest_path)
                    FileUtils.mkdir_p File.dirname(dest_path)
                    FileUtils.ln_sf File.join(component.base_dir, path), dest_path
                end

                # Generate opaque-related stuff first, so that we see them in
                # the rest of the typelib-registry-manipulation code
                handle_opaques_generation(registry)
                generated_types = self_types

                opaque_types = opaques.map { |opdef| opdef.type }
                opaque_intermediates = opaques.map do |opdef|
                    component.find_type(opdef.intermediate)
                end

		# Do some registry mumbo-jumbo to remove unneeded types to the
                # dumped registry
		minimal_registry = self.registry.
		    minimal(preloaded_registry).
		    minimal(component.rtt_registry)
		minimal_registry = component.used_toolkits.
		    inject(minimal_registry) { |reg, tk| reg.minimal(tk.registry) }

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
                end
                Generation.save_automatic "toolkit", "#{component.name}.tlb", doc.to_xml

                # Save all the types that this specific toolkit handles
                Generation.save_automatic "toolkit", "#{component.name}.typelist",
                    self_typenames.join("\n")

                # The first array is the set of types for which convertion
                # functions are generated. The second is the set of types that
                # are actually registered into the RTT type system
                #
                # The two arrays are sorted so that we don't have to recompile
                # unnecessarily (the original sets are hashes, and therefore don't
                # have a stable order).
                converted_types = generated_types.
                    find_all { |type| !type.opaque? && !toolkit.m_type?(type) }.
                    sort_by { |type| type.name }.uniq

                # We need a special case for arrays. The issue is the following:
                # for arrays, the convertion functions take pointers as input.
                # So we generate only one convertion function for all arrays of
                # the same type, regardless of the size.
                converted_types, array_types = converted_types.
                    partition { |type| !(type < Typelib::ArrayType) }

                array_types = array_types.
                    delete_if do |type|
                        component.used_toolkits.any? { |tk| tk.has_array_of?(type.deference) }
                    end.
                    inject(Hash.new) { |h, type| h[type.deference.name] = type; h }.
                    values.
                    sort_by { |type| type.name }

                registered_types = if type_export_policy == :all
                                       generated_types.dup
                                   elsif type_export_policy == :used
                                       used_types = component.self_tasks.map(&:interface_types).
                                           map(&:to_value_set).
                                           inject(&:|).
                                           map do |t|
                                               begin registry.get(t.name)
                                               rescue Typelib::NotFound
                                               end
                                           end.compact

                                       (used_types.to_value_set & generated_types.to_value_set)
                                   elsif type_export_policy == :selected
                                       selected_types.dup
                                   end

                registered_types = registered_types.
                    find_all { |type| !toolkit.m_type?(type) && !(type <= Typelib::ArrayType) }.
                    sort_by { |type| type.name }.uniq

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

                # Generate the explicit instanciation files. We split them on a
                # by-type basis, as it allows to use parallel build *and* avoid
                # memory explosion by GCC
                @template_instanciation_files = registered_types.each_with_index.map do |type, i|
                    instanciation = Generation.render_template 'toolkit/TemplateInstanciation.cpp', binding
                    out_name =  "#{component.name}TemplateInstanciation#{i}.cpp"
                    Generation.save_automatic("toolkit", out_name, instanciation)
                    out_name
                end

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

                # Generate the opaque-related stuff
                if !opaques.empty?
                    intermediates_hpp = Generation.render_template 'toolkit/ToolkitIntermediates.hpp', binding
                    Generation.save_automatic("toolkit", "#{component.name}ToolkitIntermediates.hpp", intermediates_hpp)
                    intermediates_cpp = Generation.render_template 'toolkit/ToolkitIntermediates.cpp', binding
                    Generation.save_automatic("toolkit", "#{component.name}ToolkitIntermediates.cpp", intermediates_cpp)
                    if has_opaques_with_templates?
                        user_hh = Generation.render_template 'toolkit/ToolkitUser.hpp', binding
                        user_cc = Generation.render_template 'toolkit/ToolkitUser.cpp', binding
                        Generation.save_user 'toolkit', "#{component.name}ToolkitUser.hpp", user_hh
                        Generation.save_user 'toolkit', "#{component.name}ToolkitUser.cpp", user_cc
                    end
                end

                # Finished, create the timestamp file
                Generation.touch File.join(Generation::AUTOMATIC_AREA_NAME, 'toolkit', 'stamp')
	    end
	end
    end
end

