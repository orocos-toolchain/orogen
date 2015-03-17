require 'pathname'
require 'utilrb/pkgconfig'
require 'utilrb/kernel/load_dsl_file'
require 'metaruby/dsls/doc'

module OroGen
    module Gen
    module RTT_CPP
        def self.extended_states=(value);  @extended_states = value end
        def self.extended_states_enabled?; @extended_states end
        @extended_states = true

        def self.define_default_deployments=(value);  @define_default_deployments = value end
        def self.define_default_deployments_enabled?; @define_default_deployments end
        @define_default_deployments = true

        def self.default_deployment_name(task_model_name)
            "orogen_default_#{task_model_name.gsub(/[^\w]/, '_')}"
        end

        class << self
            attr_reader :default_type_export_policy
            def default_type_export_policy=(value)
                if ![:all, :used].include?(value)
                    raise ArgumentError, "the default type export policy must be either 'all' or 'used', got '#{value}'"
                end
                @default_type_export_policy = value
            end
        end
        @default_type_export_policy = :all

        # Saved set of command line options
        class << self
            attr_accessor :command_line_options
        end
        @command_line_options = Array.new

        # Directory in which the generation command (orogen or typegen) has been
        # run
        class << self
            attr_accessor :generation_directory
        end
        @generation_directory = nil

        # Directory in which the generation command (orogen or typegen) has been
        # run, relative from the generated directory
        class << self
            attr_accessor :relative_generation_directory
        end
        @relative_generation_directory = nil

        def self.orocos_target=(target)
            @orocos_target = target.to_s
        end

        @orocos_target = nil
        def self.orocos_target
            user_target = ENV['OROCOS_TARGET']
            if @orocos_target
                @orocos_target.dup
            elsif user_target && !user_target.empty?
                user_target
            else
                'gnulinux'
            end
        end

        # This is the root class for all oroGen features: one Project instance
        # represents one oroGen project specification.
        #
        # Toplevel statements in the .orogen files are instance methods on a
        # Project instance. For instance, the
        #   
        #   task_context "Name" do
        #      ...
        #   end
        #
        # call is actually a call to #task_context.
        #
        # An existing orogen file can be loaded with Project.load
	class Project
            # A set of TaskContext instances listing all the tasks whose
            # definition is available in this project. This includes the task
            # definitions imported from other task libraries.
            #
            # See #self_tasks for the set of tasks defined in this project.
	    attr_reader :tasks

            # A set of TaskContext instances listing the tasks defined in this
            # project. 
            #
            # See #tasks for the set of all task definitions available in the
            # project.
            attr_reader :self_tasks

            # The Typelib::Registry object holding all known types defined in
            # this project
	    attr_reader :registry

            # If true, #find_type will create a new Null type when unknown types
            # are found. This is used to use orogen specifications before
            # writing down the projects.
            attr_predicate :define_dummy_types?, true

            # The target operating system for orocos. Uses the OROCOS_TARGET
            # environment variable, if set, and defaults to gnulinux otherwise.
            def orocos_target
                RTT_CPP.orocos_target.dup
            end

            def to_s
                "#<#{self.class.name}: #{name}>"
            end
            def inspect; to_s end

            # True if the orocos target is gnulinux
            def linux?; orocos_target == 'gnulinux' end
            # True if the orocos target is xenomai
            def xenomai?; orocos_target == 'xenomai' end

            # :method: version
            #
	    # The version number of this project. Defaults to "0.0"

            # :method: version 'new_version'
            #
            # Sets the version number of this project. The default is "0.0"

	    dsl_attribute(:version) do |name|
		name = name.to_s
		if name !~ /^\d/
		    raise ArgumentError, "version strings must start with a number (had: #{name})"
		end
		name
	    end

            # The set of pkg-config dependencies we depend on
            attr_reader :used_libraries
            # The set of pkg-config dependencies we depend on
            attr_reader :typekit_libraries

            # A set of ImportedProject objects describing libraries which define
            # tasks. They have to provide a .orogen file which lists the tasks
            # and their properties.
            attr_reader :used_task_libraries

            # The definition file: the full path to the .orogen file which has been
            # loaded to define this object
            #
            # nil if no file has been used
            attr_reader :deffile

            # Sets the project's definition file. It has to be an absolute path
            def deffile=(path)
                @deffile = path
                if typekit && path
                    base_dir = self.base_dir
                    typekit.base_dir = base_dir
                    if base_dir
                        typekit.user_dir      = File.join(base_dir, 'typekit')
                        typekit.templates_dir = File.join(base_dir, 'templates', 'typekit')
                        typekit.automatic_dir = File.join(base_dir, AUTOMATIC_AREA_NAME, 'typekit')
                    end
                end
            end

            # If set, the directory in which the .orogen file is. This is used
            # to update include paths for instance.
            def base_dir
                if deffile
                    File.dirname(deffile)
                end
            end

            # Create a new Project object by loading the given orogen
            # specification file
            def self.load(file, verbose = true)
                project = new
                project.load(file, verbose)
                project
            end

            # Adds some max size specification for a given type
            #
            # See OutputPort#max_sizes for a complete description of this functionality.
            # The sizes specified through this global method are applied on every port
            # of the provided type
            def max_sizes(typename = nil, *values, &block)
                if !typename && values.empty?
                    return @max_sizes
                end

                type  = find_type(typename)
                # Cannot completely validate the spec, since we may not yet have
                # the m-types. Do what we can, we'll do full blown validation
                # later
                sizes = Spec::Port.validate_max_sizes_spec(nil, values)
                @max_sizes[type.name].merge!(sizes, &block)
            end

            def validate_max_sizes_spec
                max_sizes.dup.each do |type, sizes|
                    type = intermediate_type_for(type)
                    sizes = Spec::Port.validate_max_sizes_spec(type, sizes)
                    max_sizes[type.name].merge!(sizes)
                end
            end

            @@standard_tasks = nil

            # The set of standard project defined by RTT and OCL. They are
            # defined as orogen-specification in the <tt>rtt.orogen</tt> and
            # <tt>ocl.orogen</tt>, present in orogen source code.
            def self.standard_tasks
                if @@standard_tasks
                    @@standard_tasks
                else
                    @@standard_tasks = []
                    OroGen::Loaders::RTT.standard_projects.each do |project|
                        def project.orogen_project?; false end
                        @@standard_tasks.concat project.tasks.values
                    end
                end

                @@standard_tasks
            end

            ##
            # :method:orogen_project?
            #
            # True if this class represents an oroGen project, false otherwise
            attr_predicate :orogen_project, true

            def imported?
                false
            end

            # The set of deployments known so far, as a mapping from the
            # deployment name to the Spec::Deployment instance that describes it
            attr_reader :loaded_deployments

	    def initialize
                @tasks = Hash.new
		Project.standard_tasks.each do |t|
                    @tasks[t.name] = t
                end
                @orogen_project = true
                @self_tasks = []
                @known_typekits = Hash.new
                @known_projects = Hash.new

                @name    = nil
		@version = "0.0"
		@used_typekits  = []
                @used_libraries = []
                @typekit_libraries = []
                @used_task_libraries = Set.new
                @typekit = nil

                @deployers = []

                @loaded_orogen_projects = Hash.new
                @loaded_typekits = Hash.new
                @enabled_transports = Set.new
                @opaques = Array.new
                @loaded_deployments = Hash.new

		# Load orocos-specific types which cannot be used in the
		# project-defined typekit but can be used literally in argument
		# lists or property types
		@registry = Typelib::Registry.new
                @opaque_registry = Typelib::Registry.new
                Typelib::Registry.add_standard_cxx_types(registry)
                Project.using_rtt_typekit(self)

                @max_sizes = Hash.new { |h, k| h[k] = Hash.new }
	    end

            def self.using_rtt_typekit(obj)
                OroGen::Loaders::RTT.standard_typekits.each do |tk|
                    if tk.name == 'orocos'
                        tk.extend RTTTypekit
                    end
                    obj.using_typekit(tk)
                end
            end

            # Returns the TaskContext object for the default task contexts
            # superclass (i.e. RTT::TaskContext)
            def default_task_superclass
                Project.standard_tasks.find { |t| t.name == "RTT::TaskContext" }
            end

            # Returns the typekit object that corresponds to +typekit_name+, or
            # nil if there is none
            def find_typekit(typekit_name)
                project.used_typekits.find { |tk| tk.name == typekit_name }
            end

            # The set of typekits that are already loaded on this oroGen project
            attr_reader :loaded_typekits

            # The set of task libraries that are already loaded on this oroGen
            # project
            attr_reader :loaded_orogen_projects

            class MissingTaskLibrary < LoadError
                attr_reader :name
                def initialize(name)
                    @name = name
                    super()
                end
            end

            # call-seq:
            #   orogen_project_description(name) => pkg, description
            #
            # Returns the orogen description for the oroGen project +name+.
            #
            # In the return value, pkg is the PkgConfig object that describes
            # the project, and can be nil. +description+ is either the path to
            # the oroGen description file, or its content.
            def orogen_project_description(name)
                if description = @known_projects[name]
                    return description
                end

                pkg = begin
                          begin
                              Utilrb::PkgConfig.new "orogen-project-#{name}"
                          rescue Utilrb::PkgConfig::NotFound
                              Utilrb::PkgConfig.new "#{name}-tasks-#{orocos_target}"
                          end

                      rescue Utilrb::PkgConfig::NotFound
                          raise MissingTaskLibrary.new(name), "no task library named '#{name}' is available"
                      end

                @known_projects[name] = [pkg, pkg.deffile]
            end

            # Registers an orogen file so that it can be loaded later
            # using_task_library and friends. The project name is assumed to be
            # the file basename
            def register_orogen_file(path, name = nil)
                name ||= File.basename(path, ".orogen")
                @known_projects[name] = [nil, path]
            end

            # Find the TaskContext object described by +obj+. +obj+ can either
            # be a TaskContext instance, in which case it is returned, or a
            # task context name, in which case the corresponding TaskContext
            # object is searched in the set of known ones (i.e. the ones
            # defined in this project or in the task libraries loaded
            # by #using_task_library).
            #
            # Raises ArgumentError if no such task context exists
            #
            # If the task context is defined in this project, the leading
            # namespace can be omitted. For example, in a project defined by
            #
            #   name 'myProject'
            #   task_context 'TC' do
            #   end
            #
            # the following two statements are equivalent
            #   find_task_context('TC')
            #   find_task_context('myProject::TC')
            #
            # This is not true for imported task contexts. For instance, for
            #   name 'otherProject'
            #   using_task_library 'myProject'
            #
            # the following statement will return nil:
            #   find_task_context('TC')
            # while this one works as expected:
            #   find_task_context('myProject::TC')
            def find_task_context(obj)
                task_model =
                    if obj.respond_to?(:to_str)
                        klass = tasks[obj.to_str]
                        klass || tasks["#{name}::#{obj}"]
                    else
                        obj
                    end

                if !task_model
                    raise ArgumentError, "cannot find a task context model named #{obj}"
                end
                task_model
            end

            def task_model_from_name(name)
                find_task_context(name)
            end

            # Returns true if there is a registered task context with the given
            # name, and false otherwise
            def has_task_context?(name)
                tasks.has_key?(name.to_str) || tasks.has_key?("#{self.name}::#{name}")
            end

            # Returns true if there is, in the type registry, a namespace with
            # the given name.
            def has_namespace?(name)
                if name[0] != '/'
                    name = '/' + name
                end
                if name[-1] != '/'
                    name << '/'
                end

                registry.each(name, :with_aliases => true) do |*_|
                    return true
                end
                return false
            end

            # The deployment modes that are required for this generation
            attr_reader :deployers

	    # The set of typekits that are to be used in this project. This is
            # a set of ImportedTypekit instances.
	    attr_reader :used_typekits

            # The Typelib::Registry object with only opaque definitions. This
            # does include the opaques defined in our own typekit
            attr_reader :opaque_registry

            class TypeImportError < LoadError
                attr_reader :name
                def initialize(name)
                    @name = name
                    super()
                end
            end

            # Imports the types defined by the given argument
            #
            # +name+ can either be another orogen project or a header file. In
            # the first case, the current project will be reusing the type
            # definitions and marshalling support already compiled by the other
            # project. In the second case, it will build and install a library
            # to support the new types.
            def import_types_from(name, *args)
                if has_typekit?(name)
                    using_typekit name
                else
                    typekit(true).load name, true, *args
                end
            rescue LoadError
                raise TypeImportError.new(name), "cannot find typekit or file #{name}. If this is supposed to be a header, the following include path was used: #{typekit.include_dirs.to_a.join(":")}"
            end

            def using_plugin(name)
                OroGen.load_orogen_plugin(name)
            end

            # Import an orogen-generated typekit to be used by this project.
            # The typekit is searched by name through the pkg-config tool. It
            # means that, if PREFIX is the installation prefix where the project
            # is installed, then
            #
            #   PREFIX/lib/pkgconfig
            #
            # must be listed in the PKG_CONFIG_PATH environment variable.
	    def using_typekit(typekit)
                if typekit.respond_to?(:to_str)
                    if tk = used_typekits.find { |tk| tk.name == typekit }
                        return tk
                    end
                    typekit = import_typekit(typekit)
                elsif used_typekits.include?(typekit)
                    return typekit
                end

		used_typekits << typekit
                if ours = self.typekit
                    ours.using_typekit(typekit)
                end
                if typekit.respond_to?(:project)
                    max_sizes.merge!(typekit.project.max_sizes) do |typename, a, b|
                        a.merge(b)
                    end
                end
                registry.merge(typekit.registry)
                opaque_registry.merge(typekit.opaque_registry)
                opaques.concat(typekit.opaques)
                typekit
	    end

            attr_reader :opaques

            include OpaqueHandling

            # A Typelib::Registry object defining all the types that are defined
            # in the RTT, as for instance vector<double> and string.
            def rtt_registry; Project.rtt_registry end

            # Returns the typekit object that defines this type
            def imported_typekits_for(typename)
		if typename.respond_to?(:name)
		    typename = typename.name
		end
                return used_typekits.find_all { |tk| tk.includes?(typename) }
            end

            # Returns true if +typename+ has been defined by a typekit imported
            # by using_typekit
            def imported_type?(typename)
                !imported_typekits_for(typename).empty?
            end

            # Returns true if +typename+ can be used on a task context interface
            def exported_type?(typename)
                imported_typekits_for(typename).any? do |tk|
                    tk.interface_type?(typename)
                end
            end

            # Returns the type object for +typename+, validating that we can use
            # it in a task interface, i.e. that it will be registered in the
            # RTT's typeinfo system
            def find_interface_type(typename)
                type = find_type(typename)
                if type < Typelib::NullType && define_dummy_types?
                    return type
                end

                if type < Typelib::ArrayType
                    raise ConfigError, "static arrays are not valid interface types. Use an array in a structure or a std::vector"
                end

                typekits   = imported_typekits_for(type.name)
                if !typekits.empty?
                    OroGen.debug { "#{type.name} is exported by #{typekits.map(&:name).join(", ")}" }
                end

                if !typekits.empty? && !typekits.any? { |tk| tk.interface_type?(type.name) }
                    raise ConfigError, "#{type.name}, defined in the #{typekits.map(&:name).join(", ")} typekits, is never exported"
                end
                type
            end

            def resolve_type(typename)
                find_type(typename)
            end
	    
            # Find the Typelib::Type object for +name+. +name+ can be either a
            # Typelib::Type object directly, or a type name. In both cases, the
            # type must have been defined either by the project's own typekit
            # or by the ones imported by #using_typekit
	    def find_type(typename, is_normalized = false)
		if typename
		    if typename.kind_of?(Class) && typename <= Typelib::Type
                        type = typename
                        typename = type.name
                        is_normalized = true
                    end

		    if typename.respond_to?(:to_str)
                        typename = typename.gsub('::', '/')
                        if !is_normalized
                            typename = Typelib::Type.normalize_typename(typename)
                        end
                        found_type = begin
                                         registry.build(typename)
                                     rescue Typelib::NotFound
                                         # We may need to define this type for ourselves, so
                                         # make the typekit define it ...
                                         typekit(true).find_type(typename)
                                         # ... and return our own version of it, not the
                                         # typekit's one
                                         registry.build(typename)
                                     end

                        if type && found_type != type
                            raise ArgumentError, "type definition mismatch between #{type} and #{found_type}"
                        end
                        return found_type
                    else
			raise ArgumentError, "expected a type name or a type object, got #{typename}"
		    end
		end

            rescue Typelib::NotFound => e
                if define_dummy_types?
                    return registry.create_null(typename)
                elsif typekit && typekit.respond_to?(:pending_loads) && !typekit.pending_loads.empty?
                    typekit.perform_pending_loads
                    retry
                end
                raise e.class, e.message, e.backtrace
	    end

            def extended_state_support?
                self_tasks.any? { |t| t.extended_state_support? }
            end

            # Generate the project's source files
	    def generate
		unless name
		    raise ArgumentError, "you must set a name for this project"
		end

                if name !~ /^[a-z][a-z0-9\_]+$/
                    raise ConfigError, "invalid name '#{name}': names must be all lowercase, can contain alphanumeric characters and underscores and start with a letter"
                end
                if !deffile
                    raise ArgumentError, "there is no orogen file for this project, cannot generate"
                end

		# For consistency in templates
		project = self

                # First, generate a to-be-installed version of the orogen file.
                # We do that to add command-line options like corba
                # enable/disable and extended state support.
                if File.file?(deffile)
                    orogen_file = RTT_CPP.render_template "project.orogen", binding
                    orogen_file = RTT_CPP.save_automatic(File.basename(deffile), orogen_file)
                    # In any case, touch the target file for the sake of
                    # the check-uptodate targets
                    FileUtils.touch(orogen_file)
                else
                    dir = AUTOMATIC_AREA_NAME
                    FileUtils.mkdir_p dir
                    FileUtils.touch(File.join(dir, File.basename(deffile)))
                end

		# The typekit and the task libraries populate a fake
		# installation directory .orogen/<project_name> so that the
		# includes can be referred to as <project_name>/taskNameBase.hpp.
                #
		# We have first to remove the way orogen was doing it before,
		# and then let typekit and task library do what they have to do
                fake_install_dir = File.join(project.base_dir, AUTOMATIC_AREA_NAME, project.name)
                if File.symlink?(fake_install_dir)
                    FileUtils.rm_f fake_install_dir
                    Dir.glob(File.join(project.base_dir, AUTOMATIC_AREA_NAME, "tasks", "*")).each do |path|
                        FileUtils.rm_f path if File.symlink?(path)
                    end
                end

                # This piece of code is a header-only definition that is generic
                # and should be usable in any orogen project
                #
                # (I know, this is ugly)
                # typelib_marshaller = Generation.render_template "typekit/TypelibMarshaller.hpp", binding
		# Generation.save_automatic("typekit/TypelibMarshaller.hpp", typelib_marshaller)
                # typelib_marshaller = Generation.render_template "typekit/TypelibMarshallerBase.hpp", binding
		# Generation.save_automatic("TypelibMarshallerBase.hpp", typelib_marshaller)
                # typelib_marshaller = Generation.render_template "typekit/TypelibMarshallerBase.cpp", binding
		# Generation.save_automatic("typekit/TypelibMarshallerBase.cpp", typelib_marshaller)

                # Generate the state enumeration types for each of the task
                # contexts, and load it
                if self_tasks.any?(&:extended_state_support?)
                    state_types = Generation.render_template(
                        "tasks", "TaskStates.hpp", binding)
                    header = Generation.save_automatic(
                        'typekit', 'types', project.name, "TaskStates.hpp", state_types)
                    typekit(true).load(header)
                end

		if typekit
		    typekit.generate
		end
                validate_max_sizes_spec

                pc = Generation.render_template "project.pc", binding
                Generation.save_automatic "orogen-project-#{name}.pc.in", pc

		if !self_tasks.empty?
		    self_tasks.each { |t| t.generate }

		    deployer = Generation.render_template "tasks", "DeployerComponent.cpp", binding
		    Generation.save_automatic "tasks", "DeployerComponent.cpp", deployer
		    pc = Generation.render_template "tasks", "tasks.pc", binding
		    Generation.save_automatic "tasks", "#{name}-tasks.pc.in", pc
		end

                ignorefile = Generation.render_template "gitignore", binding
                Generation.save_user ".gitignore", ignorefile

                doxygen = Generation.render_template "Doxyfile.in", binding
                Generation.save_user "Doxyfile.in", doxygen

                if !deployers.empty?
                    deployers.each { |t| t.generate }
                end

		generate_build_system
                Generation.cleanup_dir(Generation::AUTOMATIC_AREA_NAME)
		self
	    end

            CMAKE_GENERATED_CONFIG = %w{Base.cmake TaskLib.cmake}

            # Generate the CMake build system for this project
	    def generate_build_system # :nodoc:
		project = self

		FileUtils.mkdir_p File.join(Generation::AUTOMATIC_AREA_NAME, 'config')
                target_dir = Generation::AUTOMATIC_AREA_NAME
                Dir.glob File.join(Generation.template_path('config'), '*') do |path|
                    basename    = File.basename(path)
                    if !CMAKE_GENERATED_CONFIG.include?(basename)
                        Generation.save_automatic 'config', basename, File.read(path)
                    end
                end

		# Generate the automatic part of the root cmake configuration
                CMAKE_GENERATED_CONFIG.each do |file|
                    cmake = Generation.render_template "config", file, binding
                    Generation.save_automatic "config", "#{name}#{file}", cmake
                end

                if !self_tasks.empty?
                    cmake = Generation.render_template 'tasks', 'CMakeLists.txt', binding
                    Generation.save_user('tasks', "CMakeLists.txt", cmake)
                end

                cmake = Generation.render_template 'CMakeLists.txt', binding
                Generation.save_user("CMakeLists.txt", cmake)
	    end

            # Computes the set of task libraries that our own task library
            # depends on
            def tasklib_used_task_libraries
                result = self_tasks.inject(Set.new) do |set, task|
                    set | task.used_task_libraries
                end
                result.to_a.sort_by(&:name)
            end

            # Returns a list of BuildDependency object that represent the
            # dependencies for the task library
            def tasklib_dependencies
                # Get the set of typekits that we directly depend on, because
                # some of our task classes use their types in their interface.
                used_typekits = self_tasks.inject(Set.new) do |set, task|
                    set | task.used_typekits.find_all { |tk| !tk.virtual? }.map(&:name)
                end

                used_libraries = self.used_libraries.map(&:name)
                used_tasklibs = tasklib_used_task_libraries.map(&:name)

                # Cover the package names into BuildDependency objects,
                # first for the direct dependencies. Then, we look into the
                # indirect dependencies in type_includes, remove duplicates
                # and finish
                used_typekits.map! do |name|
                    build_dep = BuildDependency.new("#{name}_TYPEKIT", "#{name}-typekit-#{orocos_target}")
                    build_dep.in_context('core', 'include')
                    build_dep.in_context('core', 'link')
                end
                used_libraries.map! do |name|
                    build_dep = BuildDependency.new(name, name)
                    build_dep.in_context('core', 'include')
                    build_dep.in_context('core', 'link')
                end
                used_tasklibs.map! do |name|
                    build_dep = BuildDependency.new(
                        "#{name}_TASKLIB",
                        "#{name}-tasks-#{orocos_target}")
                    build_dep.in_context('core', 'include')
                    build_dep.in_context('core', 'link')
                end
                result = (used_typekits + used_libraries + used_tasklibs)

                var_names = result.map(&:var_name).to_set
                if typekit
                    typekit.dependencies.each do |dep|
                        next if !dep.in_context?('core') || var_names.include?(dep.var_name)
                        dep = dep.dup
                        dep.remove_context('link')
                        result << dep
                    end
                end

                result.to_set.to_a.sort_by { |dep| dep.var_name }
            end

	    # call-seq:
	    #   name(new_name) => self
            #   name => current_name
	    #
	    # Sets the project name for this generation
	    dsl_attribute :name do |new|
                if !new.respond_to?(:to_str)
                    raise ArgumentError, 'name should be a string'
                end

		if typekit && !typekit.name
		    typekit.name = new
		end
		new
	    end

            # Returns true if the following pkg-config package is available on
            # this system
            def has_library?(name)
                Utilrb::PkgConfig.new(name)
                true
            rescue Utilrb::PkgConfig::NotFound => e
                false
            end

            # call-seq:
            #   using_library 'name' => self
            #
            # Make the project build-depend on the pkg-config package +name+.
            # This is done through the use of the pkg-config tool, so you need
            # the external dependencies of your project to provide the
            # necessary files (and those files to be in a directory listed in
            # the +PKG_CONFIG_PATH+ environment variable).
            #
            # This library will be linked to both the project's typekit (if any)
            # and its task library. This is so because the library may contain
            # the implementation part of types that are exported by the typekit.
            #
            # To reduce the link interface, you may use the :typekit option to
            # avoid linking the library to the typekit:
            #
            #   using_library 'name', :typekit => false
            #
            def using_library(name, options = Hash.new)
                options = Kernel.validate_options options, :typekit => true, :typekit_link => nil
                pkg = Utilrb::PkgConfig.new(name)
                used_libraries << pkg

                if options[:typekit_link].nil?
                    options[:typekit_link] = options[:typekit]
                end

                do_typekit = options[:typekit] || options[:typekit_link]
                if do_typekit
                    typekit_libraries << [pkg, options[:typekit_link]]
                end

                if self.typekit && do_typekit
                    self.typekit.using_library(pkg, :link => options[:typekit_link])
                end
                self
            rescue Utilrb::PkgConfig::NotFound => e
                if e.name == name 
                    raise ConfigError, "no library named '#{name}' available", e.backtrace
                else
                    raise ConfigError, "a dependency of library '#{name}' is not available: library '#{e.name}' could not be found", e.backtrace
                end
            end

            def import_typekit(name)
                if tk = loaded_typekits[name]
                    return tk
                end

                pkg, registry_xml, typelist_txt = orogen_typekit_description(name)
                begin
                    loaded_typekits[name] = ImportedTypekit.
                        from_raw_data(self, name, pkg, registry_xml, typelist_txt)
                rescue Exception => e
                    raise e, "failed to import typekit #{name}: #{e.message}", e.backtrace
                end
            end

	    def register_typekit(name, registry_xml, typelist)
	    	@known_typekits[name] = [nil, registry_xml, typelist]
	    end

	    # call-seq:
	    #   project.typekit do
	    #      ... typekit setup ...
	    #   end => typekit
	    #   project.typekit => current typekit or nil
	    #
            # The first form associates a type typekit for project, as a
            # Typekit intance. The given block can set up this Typekit instance
            # by calling any instance method defined on it.
	    #
            # The second form returns a Typekit object if one is defined, and
            # nil otherwise.
	    def typekit(create = nil, &block)
                if create.nil?
                    create = true if block_given?
                end
                if create && !@typekit
                    @typekit = Typekit.new(self)

                    typekit.name     = name
                    typekit.version  = version
                    typekit.base_dir = base_dir
                    typekit.type_export_policy RTT_CPP.default_type_export_policy
                    if base_dir
                        typekit.user_dir      = File.join(base_dir, 'typekit')
                        typekit.templates_dir = File.join(base_dir, 'templates', 'typekit')
                        typekit.automatic_dir = File.join(base_dir, AUTOMATIC_AREA_NAME, 'typekit')
                    end

                    @enabled_transports.each do |t|
                        typekit.enable_plugin(t)
                    end

                    typekit.include_dirs |=
                        used_task_libraries.map { |pkg| pkg.include_dirs }.
                        flatten.to_set

                    typekit_libraries.each do |tk, do_link|
                        typekit.using_library(tk, :link => do_link)
                    end

                    # Initialize the typekit's imported_types registry
                    used_typekits.each do |tk|
                        typekit.using_typekit(tk)
                    end
                end

		if !block_given?
		    return @typekit
                else
                    @typekit.instance_eval(&block)
                    nil
		end
	    end

            # Sets or reads the typekit's type export policy.
            #
            # EXPERIMENTAL
            #
            # See Typekit#type_export_policy
            def type_export_policy(*args)
                if !typekit(false)
                    raise ConfigError, "using type_export_policy here makes no sense since no new types are defined in this project"
                end
                typekit(false).type_export_policy(*args)
            end

            # Explicitely selects types that should be added to the RTT type
            # system.
            #
            # EXPERIMENTAL
            #
            # See Typekit#export_types
            def export_types(*args)
                if !typekit(false)
                    raise ConfigError, "using export_types here makes no sense since no new types are defined in this project"
                end
                typekit(false).export_types(*args)
            end

            attr_writer :extended_states

            def extended_states?
                if @extended_states.nil? then Generation.extended_states_enabled?
                else @extended_states
                end
            end

            attr_writer :define_default_deployments

            def define_default_deployments?
                if @define_default_deployments.nil? then Generation.define_default_deployments_enabled?
                else @define_default_deployments
                end
            end

            # Creates a new task context class of this name. The generated
            # class is defined in the project's namespace. Therefore
            #
            #   name "test_project"
            #   task_context "SpecificTask" do
            #     .. task context specification ..
            #   end
            #
            # defines a <tt>test_project::SpecificTask</tt> class.
            #
            # Task contexts are represented as instances of TaskContext. See
            # the documentation of that class for more details.
	    def task_context(name, options = Hash.new, &block)
                if name == self.name
                    raise ArgumentError, "a task cannot have the same name as the project"
                elsif name !~ /^(\w+::)*\w+$/
                    raise ArgumentError, "task names need to be valid C++ identifiers, i.e. contain only alphanumeric characters and _ (got #{name})"
                end

                name = OroGen.verify_valid_identifier(name)

                # If we have a typekit, resolve all pending loads
                if typekit
                    typekit.perform_pending_loads
                end


                task = external_task_context(name, options) do
                    Spec::TaskContext.apply_default_extensions(self)
                    if block_given?
                        instance_eval(&block)
                    end
                end
                if extended_states?
                    task.extended_state_support
                end

                if !task.abstract?
                    if define_default_deployments? && enabled_transports.include?('corba')
                        simple_deployment(Generation.default_deployment_name(task.name), task.name)
                    end
                end

                task
	    end

            # Declares a task context that is being imported, not defined
            #
            # @options options [Class] type (Spec::TaskContext) the
            #   class of the created task context
            def external_task_context(name, options = Hash.new, &block)
		if has_task_context?(name)
		    raise ArgumentError, "there is already a #{name} task"
                elsif has_namespace?(name)
		    raise ArgumentError, "there is already a namespace called #{name}, this is not supported by orogen"
		end

                options = Kernel.validate_options options,
                    :class => Spec::TaskContext

		new_task = options[:class].new(self, "#{self.name}::#{name}")
                Spec.load_documentation(new_task, /^task_context/)
		new_task.instance_eval(&block) if block_given?
		tasks[new_task.name] = new_task
                self_tasks << new_task
		new_task
            end

            # Loads the oroGen project +name+
            #
            # The returned value is an instance of ImportedProject
            def load_orogen_project(name, options = Hash.new)
                name = name.to_str
                if lib = loaded_orogen_projects[name]
                    return lib
                end
                options = Kernel.validate_options options,
                    :define_dummy_types => false

                pkg, description = orogen_project_description(name)

                OroGen.info "loading oroGen project #{name} on #{self.name}"
                lib = ImportedProject.new(self, pkg)
                lib.define_dummy_types = options[:define_dummy_types]
                if File.file?(description)
                    lib.load(description)
                else
                    lib.eval(name, description)
                end

                register_loaded_project(lib.name, lib)
                lib
            end

            # Finds the specification for the deployment +name+
            def load_orogen_deployment(name)
                if deployment = loaded_deployments[name]
                    return deployment
                end

                begin
                    pkg = Utilrb::PkgConfig.new("orogen-#{name}")
                rescue Utilrb::PkgConfig::NotFound
                    raise ArgumentError, "there is no deployment called '#{name}'"
                end

                tasklib = load_orogen_project(pkg.project_name)
                deployment = tasklib.deployers.find { |d| d.name == name }
                if !deployment
                    raise InternalError, "cannot find the deployment called #{name} in #{tasklib}. Candidates were #{tasklib.deployers.map(&:name).join(", ")}"
                end
                loaded_deployments[name] = deployment
            end
            
            # Called to store a loaded project for reuse later
            def register_loaded_project(name, obj)
                OroGen.info "registering oroGen project #{name}" 
                loaded_orogen_projects[name] = obj
            end

            # Loads the task library +name+
            #
            # The returned value is an instance of ImportedProject
            def load_task_library(name, options)
                if !options.kind_of?(Hash)
                    options = { :validate => options }
                end
                options = Kernel.validate_options options,
                    :validate => true, :define_dummy_types => false

                tasklib = load_orogen_project(name, :define_dummy_types => options[:define_dummy_types])
                if options[:validate] && tasklib.self_tasks.empty?
                    raise ConfigError, "#{name} is an oroGen project, but it defines no task library"
                end
                tasklib
            end
            
            # Returns the description information for the given typekit
            def orogen_typekit_description(name)
                if description = @known_typekits[name]
                    return description
                end

                pkg = begin
                          Utilrb::PkgConfig.new("#{name}-typekit-#{orocos_target}")
                      rescue Utilrb::PkgConfig::NotFound => e
                          raise ConfigError, "no typekit named '#{name}' is available (could not load pkg-config info #{name}-typekit-#{orocos_target} in #{ENV['PKG_CONFIG_PATH']})"
                      end

                registry = File.read(pkg.type_registry)
                typelist = File.join(File.dirname(pkg.type_registry), "#{name}.typelist")
                typelist = File.read(typelist)
                @known_typekits[name] = [pkg, registry, typelist]
            end

            # Returns the ImportedTypekit object that is representing an installed
            # typekit.
            def load_typekit(name)
                name = name.to_str
                if tk = loaded_typekits[name]
                    return tk
                end

                pkg, registry_xml, typelist_txt = orogen_typekit_description(name)
                loaded_typekits[name] = ImportedTypekit.
                    from_raw_data(self, name, pkg, registry_xml, typelist_txt)
            end

            # Returns true if +name+ is a known typekit on this system
            #
            # This can be used to make the definition of parts of the oroGen
            # project conditional
            def has_typekit?(name)
                orogen_typekit_description(name)
                true
            rescue ConfigError
                false
            end

            # Returns true if +name+ is a valid task library on this system
            #
            # This can be used to make the definition of parts of the oroGen
            # project conditional
            def has_task_library?(name)
                orogen_project_description(name)
                true
            rescue MissingTaskLibrary
                false
            end

            def using_project(name)
                using_task_library(name, false)
            end

            # Declares that this project depends on task contexts defined by
            # the given orogen-generated project. After this call, the
            # definitions of the tasks in the task library are available as
            # 'name::task_context_name'
            #
            # As for #using_library, the project is searched by name by the
            # pkg-config tool. It means that, if PREFIX is the installation
            # prefix where the project is installed, then
            #
            #   PREFIX/lib/pkgconfig
            #
            # must be listed in the PKG_CONFIG_PATH environment variable.
            def using_task_library(name, options = Hash.new)
                if !options.kind_of?(Hash)
                    options = { :validate => options }
                end
                options = Kernel.validate_options options,
                    :validate => true, :define_dummy_types => false


                if name.respond_to?(:to_str)
                    if File.file?(name) && File.extname(name) == ".orogen"
                        register_orogen_file(name)
                        name = File.basename(name, ".orogen")
                    end

                    if tasklib = used_task_libraries.find { |lib| lib.name == name }
                        return tasklib
                    end
                    tasklib = load_task_library(name, options)
                else
                    tasklib = name
                end

                tasklib.self_tasks.each do |t|
                    tasks[t.name] = t
                end
                used_task_libraries << tasklib
                if self.typekit
                    typekit.using_library(tasklib.tasklib_pkg_name, :link => false)
                end

                max_sizes.merge!(tasklib.max_sizes) do |typename, a, b|
                    a.merge(b)
                end

                # Now import the typekits the project also imports, and the
                # tasklib's own typekit if there is one
                if tasklib.typekit
                    using_typekit tasklib.typekit
                end
                tasklib.used_typekits.each do |tk|
                    using_typekit tk
                end
                tasklib
            end

	    # DEPRECATED. Use #deployment instead
            def static_deployment(&block)
                OroGen::Gen::RTT_CPP.warn "static_deployment is deprecated, use #deployment(name) instead"
                OroGen::Gen::RTT_CPP.warn "static_deployment now generates a deployment called test_#{name} that is *not* part of the installation"
		deployment = deployment("test_#{name}", &block)
		deployment.do_not_install
		deployment
            end

            # True if there is a deployment with the given name in this oroGen
            # project
            def has_deployment?(name)
                deployers.any? { |d| d.name == name }
            end

            # call-seq:
            #   deployment(name[, options]) do
            #       specification
            #   end
            #
	    # Defines a deployment, i.e. an Unix executable in which a certain
	    # number of TaskContext are instanciated, associated with threads
	    # and triggers and (optionally) connected to each other and/or
	    # started.
	    #
            # The statements in the given block are method calls to a
            # StaticDeployment instance, so see the documentation of that class
            # for more information.
	    def deployment(name, &block) # :yield:
                # If we have a typekit, resolve all pending loads
                if typekit
                    typekit.perform_pending_loads
                end

                if has_deployment?(name)
                    raise ArgumentError, "there is already a deployment named '#{name}' in this oroGen project"
                end

                deployer = Spec::Deployment.new(self, name, &block)
                @enabled_transports.each do |t|
                    deployer.enable_transport(t)
                end
                deployer.instance_eval(&block) if block_given?

                # If the deployer is meant to be installed, check that there is
                # not alreay a deployment with that name
                # if deployer.install?
                #     begin
                #         pkg = Utilrb::PkgConfig.new("orogen-#{name}")
                #         if pkg.project_name != project.name
                #             raise ArgumentError, "#{name} is a deployment already used in #{pkg.project_name}"
                #         end
                #     rescue Utilrb::PkgConfig::NotFound
                #     end
                # end

                @deployers << deployer
                deployer
	    end

            # Returns the deployment model with the given name
            #
            # @return [Spec::Deployment,nil] the model found, or nil if
            #   none is registered with that name
            def find_deployment_by_name(name)
                deployers.find { |obj| obj.name == name }
            end

            # call-seq:
            #   simple_deployment(name, klass) => task_context
            #
            # Create a deployment called +name+ with one task of type +klass+
            # also called +name+.
            #
            # The returned value allows to set up the task. For instance, to
            # deploy a periodic task one would do
            #
            #   simple_deployment("task", "Task").
            #       periodic(0.001)
            def simple_deployment(name, klass)
                if has_task_library?('logger')
                    using_task_library "logger"
                end

                project = self.project

                result = nil
                deployment name do
                    result = task name, klass
                    if project.has_task_library?('logger')
                        add_default_logger
                    end
                end
                result
            end

            # This is for the sake of DSL handling
            def project; self end

            # Displays the content of this oroGen project in a nice form
            def pretty_print(pp) # :nodoc:
                if !self_tasks.empty?
                    pp.text "  Task Contexts:"
                    pp.nest(4) do
                        pp.breakable
                        pp.seplist(self_tasks) do |t|
                            t.pretty_print(pp)
                        end
                    end
                end

                if !deployers.empty?
                    pp.breakable if !self_tasks.empty?
                    pp.text "  Deployers:"
                    pp.nest(4) do
                        pp.breakable
                        pp.seplist(deployers) do |d|
                            d.pretty_print(pp)
                        end
                    end
                end
            end

            # Apply the project description included in +file+ to +self+
            def load(file, verbose = true)
                self.deffile = File.expand_path(file)
                Kernel.eval_dsl_file(deffile, self, [OroGen, RTT_CPP], verbose)
                self
            end

            def eval(name, file_contents, verbose = true)
                self.deffile = "#{name}.orogen"
                Kernel.eval_dsl_file_content(deffile, file_contents, self, [OroGen, RTT_CPP], verbose)
                self
            end

            # The set of transport names that are enabled in this project
            attr_reader :enabled_transports

            # Enable the given transports
            def enable_transports(*transport_names)
                transport_names.each do |name|
                    if typekit
                        typekit.enable_plugin(name)
                    end
                    deployers.each do |d|
                        d.enable_transport(name)
                    end
                end
                @enabled_transports |= transport_names.to_set
            end

            # Enable the given transports
            def enable_extension(extensions)
                extensions.each do |ext|
                    Spec::TaskContext.default_extensions << ext
                end
            end

	end
    end
    end
end

