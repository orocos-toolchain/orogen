require 'pathname'
require 'utilrb/pkgconfig'

module Orocos
    module Generation
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
                user_target = ENV['OROCOS_TARGET']
                if user_target && !user_target.empty?
                    user_target
                else
                    'gnulinux'
                end
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
	    def corba_enabled?; @corba end
	    # Enables corba in the generated component. See #corba_enabled?.
	    def enable_corba; @corba = true end
	    # Disables corba in the generated component. See #corba_enabled?.
	    def disable_corba; @corba = false end

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

            def self.load(file)
                component = new
                component.load(file)
            end

            def load(file)
                @deffile = File.expand_path(file)

                component = self
                Kernel.eval(File.read(deffile), binding)
                self
            end

            @@standard_tasks = nil
            def self.standard_tasks
                if @@standard_tasks
                    @@standard_tasks
                else
                    @@standard_tasks = []
                    ["rtt.orogen", "ocl.orogen"].each do |orogen|
                        component = TaskLibrary.load(self, File.expand_path(orogen, File.dirname(__FILE__)))
                        @@standard_tasks.concat component.tasks
                    end
                end

                @@standard_tasks
            end

	    def initialize
		@tasks = Component.standard_tasks.dup
		@registry = Typelib::Registry.new

		@corba   = false
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

            def default_task_superclass
                find_task_context "RTT::TaskContext"
            end

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

	    # The set of toolkits that are to be used in this component
	    attr_reader :used_toolkits

	    # Import a toolkit to be used by this component
	    def using_toolkit(name)
		if used_toolkits.any? { |n, _| n == name }
		    return
		end

		pkg = Utilrb::PkgConfig.new("#{name}-toolkit-#{orocos_target}")
		toolkit_registry = Typelib::Registry.import "#{pkg.includedir}/toolkit/#{name}ToolkitTypes.hpp", "c",
                    :define => ['__orogen'], :rawflag => pkg.cflags.split(" ")

                registry.merge(toolkit_registry)

		used_toolkits << [name, toolkit_registry]
	    end
	    
	    # Find the Typelib::Type object for +name+. +name+ can be either a
	    # Typelib::Type object directly, or a type name which should be
	    # registered into the component's registry
	    def find_type(type)
		if type
		    if type.respond_to?(:to_str)
			registry.build(type.to_str)
		    elsif !type.is_a?(Class) && !(type < Typelib::Type)
			raise ArgumentError, "expected a type object, got #{type}"
		    end
		end
	    end

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

                if !deployers.empty?
                    deployers.each { |t| t.generate }
                end

		generate_build_system
		self
	    end

	    def generate_build_system
		component = self

		FileUtils.mkdir_p(Generation::AUTOMATIC_AREA_NAME)
		FileUtils.cp_r Generation.template_path('config'), Generation::AUTOMATIC_AREA_NAME
                if !File.exists?(include_symlink = File.join(Generation::AUTOMATIC_AREA_NAME, name.downcase))
                    FileUtils.ln_sf 'tasks', include_symlink
                end

		# Generate the automatic part of the root cmake file
		cmake = Generation.render_template "config", "OrocosComponent.cmake", binding
		Generation.save_automatic "config", "OrocosComponent.cmake", cmake
		
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
	    #   name(new_name) => new_name
	    #
	    # Sets the component name for this generation
	    dsl_attribute :name do |new|
		if toolkit && !toolkit.name
		    toolkit.name new
		end
		new
	    end

            # call-seq:
            #   depends_on 'name'
            #
            # Make the component build depends on the pkg-config package +name+
            def using_library(name)
                used_libraries << Utilrb::PkgConfig.new(name)
            end

	    # call-seq:
	    #   component.toolkit(toolkit_name = component.name) do
	    #      ... toolkit setup ...
	    #   end => toolkit
	    #
	    #   component.toolkit => current toolkit or nil
	    #
	    # The first form defines the type toolkit this component defines and
	    # builds a Toolkit object based what the code block does. The given
	    # code block should call Toolkit instance methods to set up that new
	    # object
	    #
	    # The second form returns a Toolkit object if one is defined, and nil
	    # otherwise.
	    def toolkit(*args, &block)
		if args.empty? && !block_given?
		    return @toolkit
		elsif args.size > 1
		    raise ArgumentError, "expected 0 or 1 arguments, got #{args.size}"
		end

		toolkit_name = args.first || self.name
		@toolkit = Toolkit.new(self, toolkit_name, &block)
	    end

	    def task_context(name, &block)
		if find_task_context(name)
		    raise ArgumentError, "there is already a #{name} task"
		end

		new_task = TaskContext.new(self, "#{self.name}::#{name}")
		new_task.instance_eval(&block) if block_given?
		tasks << new_task
		tasks.last
	    end

            # Declares that this component depends on tasks defined in the
            # given task library. After this call, the definitions of the tasks
            # in the task library are available as 'name::task_context_name'
            def using_task_library(name)
                pkg = Utilrb::PkgConfig.new "#{name}-tasks-#{orocos_target}"
                orogen = pkg.deffile

                component = TaskLibrary.load(self, orogen)
                tasks.concat component.tasks
                if component.has_toolkit?
                    using_toolkit component.name
                end
                used_task_libraries << component
            end

            def static_deployment(&block)
                deployer = StaticDeployment.new(self, &block)
                deployer.instance_eval(&block) if block_given?
                @deployers << deployer
                deployer
            end
	end

        class TaskLibrary < Component
            attr_reader :base_component

            def self.load(base, file)
                new(base).load(file)
            end

            def initialize(component)
                @base_component = component
                super()
            end

            def task_context(name, &block)
                task = super
                task.external_definition = true
                task
            end

            attr_predicate :has_toolkit?, true
            def toolkit(*args, &block)
                if args.empty? && !block_given?
                    super
                else
                    self.has_toolkit = true
                end
                nil
            end
            def generate; raise NotImplementedError end
            def generate_build_system; raise NotImplementedError end
        end
    end
end

