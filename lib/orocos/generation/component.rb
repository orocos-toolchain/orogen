require 'pathname'

module Orocos
    module Generation
	class Component
	    # The set of tasks defined for this generation
	    attr_reader :tasks

	    # The Typelib::Registry object holding all known
	    # types defined in this component
	    attr_reader :registry

	    def initialize(&block)
		@tasks = []
		@registry = Typelib::Registry.new

		# Load orocos-specific types which cannot be used in the
		# component-defined toolkit but can be used literally in argument
		# lists or property types
		registry.import File.expand_path('orocos.tlb', File.dirname(__FILE__))

		instance_eval(&block) if block_given?
	    end

	    def generate
		if toolkit
		    toolkit.generate
		end
		tasks.each do |t|
		    t.generate
		end
		generate_build_system
		self
	    end

	    def generate_build_system
		FileUtils.mkdir_p(Generation::AUTOMATIC_AREA_NAME)
		FileUtils.cp_r Generation.template_path('config'), Generation::AUTOMATIC_AREA_NAME

		component = self

		# Generate the automatic part of the root cmake file
		cmake = Generation.render_template "config", "OrocosComponent.cmake", binding
		Generation.save_automatic "config", "OrocosComponent.cmake", cmake

		# Generate CMakeLists.txt if there is one in the template directory,
		# and the target directory exists. We check first for the user area
		# and then for the automatic area
		base_template_dir = Pathname.new(Generation.template_path)
		base_template_dir.find do |path|
		    path = Pathname.new(path).relative_path_from(base_template_dir)
		    if path.basename.to_s == "CMakeLists.txt"
			cmake = Generation.render_template path.to_s, binding

			dirname = path.dirname
			if File.directory?(dirname)
			    Generation.save_user path, cmake
			elsif File.directory?(File.join(".orocos", dirname))
			    Generation.save_automatic path, cmake
			else
			    Generation.logger.info "ignoring template #{path}"
			end
		    end
		end

		# Generate the main.cpp file, which includes the ORO_main entry
		# point
		main = Generation.render_template 'main.cpp', binding
		Generation.save_automatic 'main.cpp', main
	    end

	    # call-seq:
	    #   name(new_name) => new_name
	    #
	    # Sets the component name for this generation
	    
	    dsl_attribute :name

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
		if args.empty?
		    return @toolkit
		elsif args.size > 1
		    raise ArgumentError, "expected 0 or 1 arguments, got #{args.size}"
		end

		toolkit_name = args.first.to_s
		self.name(toolkit_name) unless self.name

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
	end
    end
end

