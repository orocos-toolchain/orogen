require 'pathname'
require 'utilrb/pkgconfig'
require 'utilrb/kernel/load_dsl_file'

module Orocos
    module Generation
        def self.corba_enabled?; @corba end
        def self.enable_corba;   @corba = true end
        def self.disable_corba;  @corba = false end
        @corba = false

        def self.extended_states=(value);  @extended_states = value end
        def self.extended_states_enabled?; @extended_states end
        @extended_states = false

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

        # This is the root class for all oroGen features: one Component instance
        # represents one oroGen project specification.
        #
        # Toplevel statements in the .orogen files are instance methods on a
        # Component instance. For instance, the
        #   
        #   task_context "Name" do
        #      ...
        #   end
        #
        # call is actually a call to #task_context.
        #
        # An existing orogen file can be loaded with Component.load
	class Component
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

            # The target operating system for orocos. Uses the OROCOS_TARGET
            # environment variable, if set, and defaults to gnulinux otherwise.
            def orocos_target
                Orocos::Generation.orocos_target.dup
            end

            # True if the orocos target is gnulinux
            def linux?; orocos_target == 'gnulinux' end
            # True if the orocos target is xenomai
            def xenomai?; orocos_target == 'xenomai' end

            # :method: version
            #
	    # The version number of this component. Defaults to "0.0"

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

	    # If the generated component should start Corba support. It can be
	    # changed by #enable_corba and #disable_corba (disabled by default)
            #
            # This setting can also be changed by the command line --corba and
            # --no-corba flags. Use the #enable_corba and #disable_corba only when
            # you want to force the use or no-use of corba.
	    def corba_enabled?; @corba.nil? ? Generation.corba_enabled? : @corba end
	    # Enables corba in the generated component. See #corba_enabled?.
	    def enable_corba;   @corba = true end
	    # Disables corba in the generated component. See #corba_enabled?.
	    def disable_corba;  @corba = false end

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

            # If set, the directory in which the .orogen file is. This is used
            # to update include paths for instance.
            def base_dir
                if deffile
                    File.dirname(deffile)
                end
            end

            # Create a new Component object by loading the given orogen
            # specification file
            def self.load(file)
                component = new
                component.load(file)
                component
            end

            @@standard_tasks = nil

            # The set of standard components defined by RTT and OCL. They are
            # defined as orogen-specification in the <tt>rtt.orogen</tt> and
            # <tt>ocl.orogen</tt>, present in orogen source code.
            def self.standard_tasks
                if @@standard_tasks
                    @@standard_tasks
                else
                    @@standard_tasks = []
                    ["rtt.orogen", "ocl.orogen"].each do |orogen|
                        component = ImportedProject.load(nil, nil, File.expand_path(orogen, File.dirname(__FILE__)))
                        component.orogen_project = false
                        @@standard_tasks.concat component.tasks
                    end
                end

                @@standard_tasks
            end

            ##
            # :method:orogen_project?
            #
            # True if this class represents an oroGen project, false otherwise
            attr_predicate :orogen_project, true

	    def initialize
		@tasks = Component.standard_tasks.dup
                @orogen_project = true
                @self_tasks = []

                @name    = nil
		@corba   = nil
		@version = "0.0"
		@used_typekits  = []
                @used_libraries = []
                @typekit_libraries = []
                @used_task_libraries = Set.new
                @typekit = nil

                @deployers = []

                @loaded_orogen_projects = Hash.new
                @loaded_typekits = Hash.new

		# Load orocos-specific types which cannot be used in the
		# component-defined typekit but can be used literally in argument
		# lists or property types
                if !Component.rtt_registry
                    Component.load_rtt_registry
                end
		@registry = Typelib::Registry.new
		@registry.merge rtt_registry
	    end

            class << self
                attr_reader :rtt_registry
            end

            def self.load_rtt_registry
                rtt_tlb = File.expand_path('orocos.tlb', File.dirname(__FILE__))
                @rtt_registry = Typelib::Registry.import rtt_tlb
            end


            # Returns the TaskContext object for the default task contexts
            # superclass (i.e. RTT::TaskContext)
            def default_task_superclass
                find_task_context "RTT::TaskContext"
            end

            # The set of typekits that are already loaded on this oroGen project
            attr_reader :loaded_typekits

            # The set of task libraries that are already loaded on this oroGen
            # project
            attr_reader :loaded_orogen_projects

            # call-seq:
            #   orogen_project_description(name) => pkg, description
            #
            # Returns the orogen description for the oroGen project +name+.
            #
            # In the return value, pkg is the PkgConfig object that describes
            # the project, and can be nil. +description+ is either the path to
            # the oroGen description file, or its content.
            def orogen_project_description(name)
                pkg = begin
                          begin
                              Utilrb::PkgConfig.new "orogen-project-#{name}"
                          rescue Utilrb::PkgConfig::NotFound
                              Utilrb::PkgConfig.new "#{name}-tasks-#{orocos_target}"
                          end

                      rescue Utilrb::PkgConfig::NotFound
                          raise ConfigError, "no task library named '#{name}' is available"
                      end

                return pkg, pkg.deffile
            end

            # Find the TaskContext object described by +obj+. +obj+ can either
            # be a TaskContext instance, in which case it is returned, or a
            # task context name, in which case the corresponding TaskContext
            # object is searched in the set of known ones (i.e. the ones
            # defined in this component or in the task libraries loaded
            # by #using_task_library).
            #
            # Raises ArgumentError if no such task context exists
            #
            # If the task context is defined in this component, the leading
            # namespace can be omitted. For example, in a component defined by
            #
            #   name 'myComponent'
            #   task_context 'TC' do
            #   end
            #
            # the following two statements are equivalent
            #   find_task_context('TC')
            #   find_task_context('myComponent::TC')
            #
            # This is not true for imported task contexts. For instance, for
            #   name 'otherComponent'
            #   using_task_library 'myComponent'
            #
            # the following statement will return nil:
            #   find_task_context('TC')
            # while this one works as expected:
            #   find_task_context('myComponent::TC')
            def find_task_context(obj)
                task_model =
                    if obj.respond_to?(:to_str)
                        klass = tasks.find { |t| t.name == obj.to_str }
                        klass || tasks.find { |t| t.name == "#{name}::#{obj}" }
                    else
                        obj
                    end

                if !task_model
                    raise ArgumentError, "cannot find a task context model named #{obj}"
                end
                task_model
            end

            # Returns true if there is a registered task context with the given
            # name, and false otherwise
            def has_task_context?(name)
                name = name.to_str
                klass = tasks.find { |t| t.name == name }
                klass || tasks.find { |t| t.name == "#{self.name}::#{name}" }
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

                registry.enum_for(:each_type).any? do |type|
                    type.namespace =~ /^#{Regexp.quote(name)}/
                end
            end

            # The deployment modes that are required for this generation
            attr_reader :deployers

	    # The set of typekits that are to be used in this component. This is
            # a set of ImportedTypekit instances.
	    attr_reader :used_typekits

            # Imports the types defined by the given argument
            #
            # +name+ can either be another orogen project or a header file. In
            # the first case, the current project will be reusing the type
            # definitions and marshalling support already compiled by the other
            # project. In the second case, it will build and install a library
            # to support the new types.
            def import_types_from(name, *args)
                if Utilrb::PkgConfig.has_package?("#{name}-typekit-#{orocos_target}")
                    using_typekit name
                else
                    typekit(true).load name
                end
                import_types_from(*args) unless args.empty?
            end

            # Import an orogen-generated typekit to be used by this component.
            # The typekit is searched by name through the pkg-config tool. It
            # means that, if PREFIX is the installation prefix where the component
            # is installed, then
            #
            #   PREFIX/lib/pkgconfig
            #
            # must be listed in the PKG_CONFIG_PATH environment variable.
	    def using_typekit(name)
		if used_typekits.any? { |tk| tk.name == name }
		    return
		end

                typekit = load_typekit(name)
		used_typekits << typekit
                if ours = self.typekit
                    ours.using_typekit(typekit)
                end
                registry.merge(typekit.registry)
	    end

            # A Typelib::Registry object defining all the types that are defined
            # in the RTT, as for instance vector<double> and string.
            def rtt_registry; Component.rtt_registry end

            # Returns true if +typename+ has been defined by a typekit imported
            # by using_typekit
            def imported_type?(typename)
		if typename.respond_to?(:name)
		    typename = typename.name
		end

                rtt_registry.includes?(typename) ||
                    used_typekits.any? { |tk| tk.includes?(typename) }
            end
	    
            # Find the Typelib::Type object for +name+. +name+ can be either a
            # Typelib::Type object directly, or a type name. In both cases, the
            # type must have been defined either by the component's own typekit
            # or by the ones imported by #using_typekit
	    def find_type(typename)
		if typename
		    if typename.kind_of?(Class) && typename <= Typelib::Type
                        type = typename
                        typename = type.name
                    end

		    if typename.respond_to?(:to_str)
                        typename = typename.gsub('::', '/')
                        typename = Typelib::Type.normalize_typename(typename)
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
                        found_type
                    else
			raise ArgumentError, "expected a type name or a type object, got #{typename}"
		    end
		end
	    end

            # Generate the component's source files
	    def generate
		unless name
		    raise ArgumentError, "you must set a name for this component"
		end

                if name !~ /^[a-z][a-z0-9\_]+$/
                    raise ConfigError, "invalid name '#{name}': names must be all lowercase, can contain alphanumeric characters and underscores and start with a letter"
                end
                if !deffile
                    raise ArgumentError, "there is no orogen file for this component, cannot generate"
                end
                if !File.file?(deffile)
                    Orocos::Generation.warn "#{deffile} does not exist, assuming that we are generating a stub component"
                end

		# For consistency in templates
		component = self

                # First, generate a to-be-installed version of the orogen file.
                # We do that to add command-line options like corba
                # enable/disable and extended state support.
                if File.file?(deffile)
                    orogen_file = Generation.render_template "project.orogen", binding
                    orogen_file = Generation.save_automatic(File.basename(deffile), orogen_file)
                    # In any case, touch the target file for the sake of
                    # the check-uptodate targets
                    FileUtils.touch(orogen_file)
                else
                    dir = Orocos::Generation::AUTOMATIC_AREA_NAME
                    FileUtils.mkdir_p dir
                    FileUtils.touch(File.join(dir, File.basename(deffile)))
                end

		# The typekit and the task libraries populate a fake
		# installation directory .orogen/<project_name> so that the
		# includes can be referred to as <project_name>/taskNameBase.hpp.
                #
		# We have first to remove the way orogen was doing it before,
		# and then let typekit and task library do what they have to do
                fake_install_dir = File.join(component.base_dir, AUTOMATIC_AREA_NAME, component.name)
                if File.symlink?(fake_install_dir)
                    FileUtils.rm_f fake_install_dir
                    Dir.glob(File.join(component.base_dir, AUTOMATIC_AREA_NAME, "tasks", "*")).each do |path|
                        FileUtils.rm_f path if File.symlink?(path)
                    end
                end

                # This piece of code is a header-only definition that is generic
                # and should be usable in any orogen component
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
                        "#{component.name}TaskStates.hpp", state_types)
                    typekit(true).load(header)
                end

		if typekit
                    typekit.name     = name
                    typekit.version  = version
                    typekit.base_dir = base_dir
                    typekit.user_dir = File.join(base_dir, 'typekit')
                    typekit.templates_dir = File.join(base_dir, 'templates', 'typekit')
                    typekit.automatic_dir = File.join(base_dir, AUTOMATIC_AREA_NAME, 'typekit')
                    if corba_enabled?
                        typekit.enable_plugin('corba')
                    end

		    typekit.generate
		end

                pc = Generation.render_template "project.pc", binding
                Generation.save_automatic "orogen-project-#{name}.pc.in", pc

		if !self_tasks.empty?
		    self_tasks.each { |t| t.generate }

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

            # Generate the CMake build system for this component
	    def generate_build_system # :nodoc:
		component = self

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
                    set | task.used_typekits.map(&:name)
                end

                used_libraries = self.used_libraries.map(&:name)
                used_tasklibs = tasklib_used_task_libraries.map(&:name)

                # Cover the package names into BuildDependency objects,
                # first for the direct dependencies. Then, we look into the
                # indirect dependencies in type_includes, remove duplicates
                # and finish
                used_typekits.map! do |name|
                    build_dep = BuildDependency.new("#{name}_TOOLKIT", "#{name}-typekit-#{orocos_target}")
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
                        next if dep.in_context?('corba') || var_names.include?(dep.var_name)
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
	    # Sets the component name for this generation
	    dsl_attribute :name do |new|
                if !new.respond_to?(:to_str)
                    raise ArgumentError, 'name should be a string'
                end

		if typekit && !typekit.name
		    typekit.name new
		end
		new
	    end

            # call-seq:
            #   using_library 'name' => self
            #
            # Make the component build-depend on the pkg-config package +name+.
            # This is done through the use of the pkg-config tool, so you need
            # the external dependencies of your component to provide the
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
                options = Kernel.validate_options options, :typekit => true
                pkg = Utilrb::PkgConfig.new(name)
                used_libraries << pkg
                if options[:typekit]
                    typekit_libraries << pkg
                end

                if self.typekit
                    self.typekit.using_library(pkg, options[:typekit])
                end
                self
            rescue Utilrb::PkgConfig::NotFound => e
                raise ConfigError, "no library named '#{name}' is available", e.backtrace
            end

	    # call-seq:
	    #   component.typekit do
	    #      ... typekit setup ...
	    #   end => typekit
	    #   component.typekit => current typekit or nil
	    #
            # The first form associates a type typekit for component, as a
            # Typekit intance. The given block can set up this Typekit instance
            # by calling any instance method defined on it.
	    #
            # The second form returns a Typekit object if one is defined, and
            # nil otherwise.
	    def typekit(create = nil, &block)
                if create.nil?
                    create = true if block_given?
                end
                if create
                    @typekit ||= Typekit.new(self)

                    # Initialize the typekit's include_dirs set
                    if base_dir
                        typekit.include_dirs << base_dir
                    end
                    typekit.include_dirs |=
                        used_task_libraries.map { |pkg| pkg.include_dirs }.
                        flatten.to_set

                    used_libraries.each do |tk|
                        typekit.using_library(tk, typekit_libraries.include?(tk))
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

            # Creates a new task context class of this name. The generated
            # class is defined in the component's namespace. Therefore
            #
            #   name "test_component"
            #   task_context "SpecificTask" do
            #     .. task context specification ..
            #   end
            #
            # defines a <tt>test_component::SpecificTask</tt> class.
            #
            # Task contexts are represented as instances of TaskContext. See
            # the documentation of that class for more details.
            #
	    def task_context(name, &block)
                task = external_task_context(name, &block)
                if extended_states?
                    task.extended_state_support
                end
                task
	    end

            # Declares a task context that is being imported, not defined
            def external_task_context(name, &block)
		if has_task_context?(name)
		    raise ArgumentError, "there is already a #{name} task"
                elsif has_namespace?(name)
		    raise ArgumentError, "there is already a namespace called #{name}, this is not supported by orogen"
		end

		new_task = TaskContext.new(self, "#{self.name}::#{name}")
		new_task.instance_eval(&block) if block_given?
		tasks << new_task
                self_tasks << new_task
		tasks.last
            end

            # Loads the oroGen project +name+
            #
            # The returned value is an instance of ImportedProject
            def load_orogen_project(name)
                name = name.to_str
                if lib = loaded_orogen_projects[name]
                    return lib
                end
                pkg, description = orogen_project_description(name)

                if File.file?(description)
                    lib = ImportedProject.load(self, pkg, description)
                else
                    lib = ImportedProject.new(self, pkg)
                    lib.eval(name, description)
                end

                register_loaded_project(name, lib)
            end
            
            # Called to store a loaded project for reuse later
            def register_loaded_project(name, obj)
                loaded_orogen_projects[name] = obj
            end

            # Loads the task library +name+
            #
            # The returned value is an instance of ImportedProject
            def load_task_library(name)
                tasklib = load_orogen_project(name)
                if tasklib.self_tasks.empty?
                    raise ConfigError, "#{name} is an oroGen project, but it defines no task library"
                end
                tasklib
            end
            
            # Returns the description information for the given typekit
            def orogen_typekit_description(name)
                pkg = begin
                          Utilrb::PkgConfig.new("#{name}-typekit-#{orocos_target}")
                      rescue Utilrb::PkgConfig::NotFound => e
                          raise ConfigError, "no typekit named '#{name}' is available"
                      end

                registry = File.read(pkg.type_registry)
                typelist = File.join(File.dirname(pkg.type_registry), "#{name}.typelist")
                typelist = File.read(typelist)
                return pkg, registry, typelist
            end

            # Returns true if +name+ has a typekit available
            def has_typekit?(name)
                Utilrb::PkgConfig.has_package?("#{name}-typekit-#{orocos_target}")
            end

            # Returns the ImportedTypekit object that is representing an installed
            # typekit.
            def load_typekit(name)
                name = name.to_str
                if tk = loaded_typekits[name]
                    return tk
                end

                pkg, registry_xml, typelist_txt = orogen_typekit_description(name)

                typekit_registry = Typelib::Registry.from_xml(registry_xml)
                typekit_typelist = typelist_txt.split("\n").map(&:chomp)

                typekit = ImportedTypekit.new(self, name,
                              pkg, typekit_registry, typekit_typelist)
                loaded_typekits[name] = typekit
                typekit
            end

            # Declares that this component depends on task contexts defined by
            # the given orogen-generated component. After this call, the
            # definitions of the tasks in the task library are available as
            # 'name::task_context_name'
            #
            # As for #using_library, the component is searched by name by the
            # pkg-config tool. It means that, if PREFIX is the installation
            # prefix where the component is installed, then
            #
            #   PREFIX/lib/pkgconfig
            #
            # must be listed in the PKG_CONFIG_PATH environment variable.
            def using_task_library(name)
		if tasklib = used_task_libraries.find { |lib| lib.name == name }
		    return tasklib
		end

                tasklib = load_task_library(name)
                tasks.concat tasklib.self_tasks
                used_task_libraries << tasklib
                if self.typekit
                    self.typekit.include_dirs |= tasklib.inclued_dirs.to_set
                end

                # Now import the typekits the component also imports, and the
                # tasklib's own typekit if there is one
                if tasklib.has_typekit?
                    using_typekit tasklib.name
                end
                tasklib.used_typekits.each do |tk|
                    using_typekit tk.name
                end
                tasklib
            end

	    # DEPRECATED. Use #deployment instead
            def static_deployment(&block)
                STDERR.puts "WARN: static_deployment is deprecated, use #deployment(name) instead"
                STDERR.puts "WARN: static_deployment now generates a deployment called test_#{name} that is *not* part of the installation"
		deployment = deployment("test_#{name}", &block)
		deployment.do_not_install
		deployment
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
                deployer = StaticDeployment.new(self, name, &block)
                deployer.instance_eval(&block) if block_given?

                # If the deployer is meant to be installed, check that there is
                # not alreay a deployment with that name
                # if deployer.install?
                #     begin
                #         pkg = Utilrb::PkgConfig.new("orogen-#{name}")
                #         if pkg.project_name != component.name
                #             raise ArgumentError, "#{name} is a deployment already used in #{pkg.project_name}"
                #         end
                #     rescue Utilrb::PkgConfig::NotFound
                #     end
                # end

                @deployers << deployer
                deployer
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
                result = nil
                deployment name do
                    result = task name, klass
                    add_default_logger
                end
                result
            end

            # This is for the sake of DSL handling
            def component; self end

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
                @deffile = File.expand_path(file)
                Kernel.eval_dsl_file(deffile, self, Orocos::Generation, false)
                self
            end

            def eval(name, file_contents)
                @deffile = "#{name}.orogen"
                Kernel.eval_dsl_file_content(deffile, file_contents, self, Orocos::Generation, false)
                self
            end
	end

	Project = Component
    end
end

