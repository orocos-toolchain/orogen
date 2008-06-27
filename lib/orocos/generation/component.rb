require 'pathname'
require 'utilrb/pkgconfig'

module Orocos
    module Generation
	class Component
	    # The set of tasks defined for this generation
	    attr_reader :tasks

	    # The Typelib::Registry object holding all known
	    # types defined in this component
	    attr_reader :registry

	    # The version number of this component. Defaults to
	    # "0.0"
	    dsl_attribute(:version) do |name|
		name = name.to_s
		if name != /^\d/
		    raise ArgumentError, "version strings must start with a number"
		end
		name
	    end

	    # If the generated component should start Corba support. It can be
	    # changed by #enable_corba and #disable_corba (enabled by default)
	    def corba_enabled?; @corba end
	    # Enables corba in the generated component. See also #disable_corba
	    # and the #corba attribute
	    def enable_corba; @corba = true end
	    # Disables corba in the generated component. See also #enable_corba
	    # and the #corba attribute.
	    def disable_corba; @corba = false end

            # The set of pkg-config dependencies we depend on
            attr_reader :used_libraries

            # If set, the directory in which the .orogen file is used. This is
            # used to update include paths for instance.
            attr_reader :base_dir

	    def initialize(base_dir = nil, &block)
		@tasks = []
		@registry = Typelib::Registry.new
                @base_dir = base_dir

		@corba   = true
		@version = "0.0"
		@used_toolkits  = []
                @used_libraries = []

                @deployers = []

		# Load orocos-specific types which cannot be used in the
		# component-defined toolkit but can be used literally in argument
		# lists or property types
		registry.import File.expand_path('orocos.tlb', File.dirname(__FILE__))

		instance_eval(&block) if block_given?
	    end

            # The deployment modes that are required for this generation
            attr_reader :deployers

	    # The set of toolkits that are to be used in this component
	    attr_reader :used_toolkits

	    # Import a toolkit to be used by this component
	    def using_toolkit(name)
		if used_toolkits.include?(name)
		    return
		end

		used_toolkits << name
		pkg = Utilrb::PkgConfig.new("#{name}-toolkit-gnulinux")
		registry.import "#{pkg.includedir}/toolkit/#{name}ToolkitTypes.hpp", "c"
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

		# For consistency in templates
		component = self

		if toolkit
		    toolkit.generate
		end

		if !tasks.empty?
		    tasks.each { |t| t.generate }

		    pc = Generation.render_template "tasks", "tasks.pc", binding
		    Generation.save_automatic "tasks", "#{name}-tasks.pc.in", pc
		end

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
		if tasks.find { |t| t.name == name }
		    raise ArgumentError, "there is already a #{name} task"
		end

		new_task = TaskContext.new(self, name, &block)
		tasks << new_task
		tasks.last
	    end

            def static_deployment(&block)
                deployer = StaticDeployment.new(self, &block)
                @deployers << deployer
                deployer
            end
	end
    end
end

