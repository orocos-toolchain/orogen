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
		FileUtils.mkdir_p('.orocos')
		FileUtils.cp_r Generation.template_path('config'), '.orocos'

		component = self

		# Generate the toplevel CMakeLists.txt
		root_cmake = Generation.render_template 'CMakeLists.txt', binding
		Generation.save_public_automatic 'CMakeLists.txt', root_cmake

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

