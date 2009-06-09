require 'pathname'
require 'utilrb/pkgconfig'

module Orocos
    module Generation
        def self.corba_enabled?; @corba end
        def self.enable_corba;   @corba = true end
        def self.disable_corba;  @corba = false end

        def self.orocos_target=(target)
            @orocos_target = target.to_s
        end

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

	class Component
	    # The set of tasks whose definition is available
	    attr_reader :tasks

            # The set of tasks defined by this component
            def self_tasks
                tasks.find_all { |t| !t.external_definition? }
            end

	    # The Typelib::Registry object holding all known
	    # types defined in this component
	    attr_reader :registry

            # The target OS for orocos. Uses the OROCOS_TARGET environment
            # variable, if set, and defaults to gnulinux otherwise.
            def orocos_target
                Orocos::Generation.orocos_target.dup
            end

            # True if the orocos target is gnulinux
            def linux?; orocos_target == 'gnulinux' end
            # True if the orocos target is xenomai
            def xenomai?; orocos_target == 'xenomai' end

	    # The version number of this component. Defaults to
	    # "0.0"
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

            # A set of TaskLibrary objects describing libraries which define
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

            # Apply the component specification included in +file+ to +self+
            def load(file)
                @deffile = File.expand_path(file)

                component = self
                Kernel.eval(File.read(deffile), binding)
                self
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
                        component = TaskLibrary.load(self, nil, File.expand_path(orogen, File.dirname(__FILE__)))
                        @@standard_tasks.concat component.tasks
                    end
                end

                @@standard_tasks
            end

	    def initialize
		@tasks = Component.standard_tasks.dup
		@registry = Typelib::Registry.new

		@corba   = nil
		@version = "0.0"
		@used_toolkits  = []
                @used_libraries = []
                @used_task_libraries = []

                @deployers = []

		# Load orocos-specific types which cannot be used in the
		# component-defined toolkit but can be used literally in argument
		# lists or property types
		registry.import File.expand_path('orocos.tlb', File.dirname(__FILE__))
	    end

            # Returns the TaskContext object for the default task contexts
            # superclass (i.e. RTT::TaskContext)
            def default_task_superclass
                find_task_context "RTT::TaskContext"
            end

            # Find the TaskContext object described by +obj+. +obj+ can either
            # be a TaskContext instance, in which case it is returned, or a
            # task context name, in which case the corresponding TaskContext
            # object is searched in the set of known ones (i.e. the ones
            # defined in this component or in the task libraries loaded
            # by #using_task_library).
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
                if obj.respond_to?(:to_str)
                    klass = tasks.find { |t| t.name == obj.to_str }
                    klass || tasks.find { |t| t.name == "#{name}::#{obj}" }
                else
                    obj
                end
            end

            # The deployment modes that are required for this generation
            attr_reader :deployers

	    # The set of toolkits that are to be used in this component. This is
            # a set of ImportedToolkit instances.
	    attr_reader :used_toolkits

            # Import an orogen-generated toolkit to be used by this component.
            # The toolkit is searched by name through the pkg-config tool. It
            # means that, if PREFIX is the installation prefix where the component
            # is installed, then
            #
            #   PREFIX/lib/pkgconfig
            #
            # must be listed in the PKG_CONFIG_PATH environment variable.
	    def using_toolkit(name)
		if used_toolkits.any? { |tk| tk.name == name }
		    return
		end

		pkg = Utilrb::PkgConfig.new("#{name}-toolkit-#{orocos_target}")
		toolkit_registry = Typelib::Registry.import pkg.type_registry

                registry.merge(toolkit_registry)
		used_toolkits << ImportedToolkit.new(name, pkg, toolkit_registry)
	    end

            # Returns true if +typename+ has been defined by a toolkit imported
            # by using_toolkit
            def imported_type?(typename)
                used_toolkits.any? { |tk| tk.includes?(typename) }
            end
	    
            # Find the Typelib::Type object for +name+. +name+ can be either a
            # Typelib::Type object directly, or a type name. In both cases, the
            # type must have been defined either by the component's own toolkit
            # or by the ones imported by #using_toolkit
	    def find_type(type)
		if type
		    if type.respond_to?(:to_str)
                        type = Typelib::Type.normalize_typename(type)
                        begin
                            registry.build(type)
                        rescue Typelib::NotFound
                            # We may need to define this type for ourselves, so
                            # make the toolkit define it ...
                            toolkit { find_type(type) }
                            # ... and return our own version of it, not the
                            # toolkit's one
                            registry.build(type)
                        end
		    elsif type.kind_of?(Class) && type <= Typelib::Type
                        type
                    else
			raise ArgumentError, "expected a type object, got #{type}"
		    end
		end
	    end

            # Generate the component's source files
	    def generate
		unless name
		    raise ArgumentError, "you must set a name for this component"
		end
                unless deffile
                    raise ArgumentError, "there is no orogen file for this component, cannot generate"
                end

		# For consistency in templates
		component = self

		if toolkit
		    toolkit.generate
		end

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
		self
	    end

            CMAKE_GENERATED_CONFIG = %w{Base.cmake TaskLib.cmake}

            # Generate the CMake build system for this component
	    def generate_build_system
		component = self

		FileUtils.mkdir_p File.join(Generation::AUTOMATIC_AREA_NAME, 'config')
                target_dir = Generation::AUTOMATIC_AREA_NAME
                Dir.glob File.join(Generation.template_path('config'), '*') do |path|
                    basename = File.basename(path)
                    target_name = File.join(Generation::AUTOMATIC_AREA_NAME, 'config', basename)
                    if !CMAKE_GENERATED_CONFIG.include?(basename) &&
                        (!File.exists?(target_name) || File.read(target_name) != File.read(path))
                        Generation.logger.info "copying #{path}"
                        FileUtils.cp path, target_name
                    end
                end

		# FileUtils.cp_r Generation.template_path('config'), Generation::AUTOMATIC_AREA_NAME
                if !self_tasks.empty? && !File.exists?(include_symlink = File.join(Generation::AUTOMATIC_AREA_NAME, name.downcase))
                    FileUtils.ln_sf 'tasks', include_symlink
                end

		# Generate the automatic part of the root cmake configuration
                CMAKE_GENERATED_CONFIG.each do |file|
                    cmake = Generation.render_template "config", file, binding
                    Generation.save_automatic "config", "#{name}#{file}", cmake
                end
		
		# Generate CMakeLists.txt if there is one in the template directory,
		# and the target directory exists. We check first for the user area
		# and then for the automatic area
		base_template_dir = Pathname.new(Generation.template_path)
		base_template_dir.find do |path|
		    path = Pathname.new(path).relative_path_from(base_template_dir)
                    if path.to_s == 'toolkit' && !toolkit
                        Find.prune
                    end

		    dirname = path.dirname
		    if path.basename.to_s == "CMakeLists.txt"
			if File.directory?(dirname)
			    cmake = Generation.render_template path.to_s, binding
			    Generation.save_user path, cmake
			elsif File.directory?(File.join(Generation::AUTOMATIC_AREA_NAME, dirname))
			    cmake = Generation.render_template path.to_s, binding
			    Generation.save_automatic path, cmake
			else
			    Generation.logger.info "ignoring template #{path}"
			end
		    elsif path.basename.to_s == "CMakeLists-auto.txt"
			if File.directory?(File.join(Generation::AUTOMATIC_AREA_NAME, dirname))
			    target = File.join(dirname, path.basename.to_s.gsub(/-auto\.txt$/, '.txt'))
			    cmake = Generation.render_template path.to_s, binding
			    Generation.save_automatic target, cmake
			else
			    Generation.logger.info "ignoring template #{path}"
			end
		    end
		end
	    end

	    # call-seq:
	    #   name(new_name) => self
            #   name => current_name
	    #
	    # Sets the component name for this generation
	    dsl_attribute :name do |new|
		if toolkit && !toolkit.name
		    toolkit.name new
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
            def using_library(name)
                used_libraries << Utilrb::PkgConfig.new(name)
                self
            end

	    # call-seq:
	    #   component.toolkit do
	    #      ... toolkit setup ...
	    #   end => toolkit
	    #   component.toolkit => current toolkit or nil
	    #
            # The first form associates a type toolkit for component, as a
            # Toolkit intance. The given block can set up this Toolkit instance
            # by calling any instance method defined on it.
	    #
            # The second form returns a Toolkit object if one is defined, and
            # nil otherwise.
	    def toolkit(&block)
		if !block_given?
		    return @toolkit
                else
                    @toolkit ||= Toolkit.new(self, &block)
                    @toolkit.instance_eval(&block)
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
		if find_task_context(name)
		    raise ArgumentError, "there is already a #{name} task"
		end

		new_task = TaskContext.new(self, "#{self.name}::#{name}")
		new_task.instance_eval(&block) if block_given?
		tasks << new_task
		tasks.last
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
                pkg = Utilrb::PkgConfig.new "#{name}-tasks-#{orocos_target}"
                orogen = pkg.deffile

                component = TaskLibrary.load(self, pkg, orogen)
                tasks.concat component.tasks
                used_task_libraries << component
            end

            # call-seq:
            #   static_deployment do
            #     ... deployment specification ...
            #   end
            #
            # Define a static deployment, i.e. the definition of TaskContext
            # instances, to which Activity instances they should be linked and
            # a set of connections.
            #
            # The deployment is represented by a StaticDeployment instance. The
            # deployment specification is made of a set of method calls on that
            # instance, so see the documentation of that class for the list of
            # possible operations.
            def static_deployment(&block)
                deployer = StaticDeployment.new(self, &block)
                deployer.instance_eval(&block) if block_given?
                @deployers << deployer
                deployer
            end
	end

        # Instances of this class represent a toolkit that has been imported
        # using Component#using_toolkit.
        class ImportedToolkit
            attr_reader :name
            attr_reader :pkg
            attr_reader :registry

            def initialize(name, pkg, registry)
                @name, @pkg, @registry = name, pkg, registry
            end
            def includes?(name)
                registry.includes?(name)
            end
        end

        # Instances of this class represent a task library loaded in a
        # component, i.e.  a set of TaskContext defined externally and imported
        # using #using_task_library.
        #
        # For the task contexts imported this way,
        # TaskContext#external_definition?  returns true.
        class TaskLibrary < Component
            # The component in which the library has been imported
            attr_reader :base_component
            # The pkg-config file defining this task library
            attr_reader :pkg

            # Import in the +base+ component the task library whose orogen
            # specification is included in +file+
            def self.load(base, pkg, file)
                new(base, pkg).load(file)
            end

            def initialize(component, pkg)
                @base_component, @pkg = component, pkg
                super()
            end

            def task_context(name, &block) # :nodoc:
                task = super
                task.external_definition = true
                task
            end

            # True if this task library defines a toolkit
            attr_predicate :has_toolkit?, true

            def toolkit(*args, &block) # :nodoc:
                if args.empty? && !block_given?
                    super
                else
                    using_toolkit name
                    self.has_toolkit = true
                end
                nil
            end

            # Task library objects represent an import, and as such they cannot
            # be generated.  This method raises NotImplementedError
            def generate; raise NotImplementedError end
            # Task library objects represent an import, and as such they cannot
            # be generated.  This method raises NotImplementedError
            def generate_build_system; raise NotImplementedError end
        end
    end
end

