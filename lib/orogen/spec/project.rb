module OroGen
    module Spec
        # Representation of an oroGen project
        class Project
            # The loader that should be used to get our dependencies
            attr_reader :loader
            # This project's typekit
            # @return [Typekit,nil]
            attr_accessor :typekit
            # This project's tasks
            # @return [Hash<String,OroGen::Spec::TaskContext>]
            attr_reader :tasks
            # This project's deployments
            # @return [Hash<String,OroGen::Spec::Deployment>]
            attr_reader :deployers
            # If true, default deployments get automatically generated for the
            # tasks in this project
            attr_predicate :define_default_deployments?, true
            # The set of transport names that are enabled in this project
            # @return [Set<String>]
            attr_reader :enabled_transports

            def initialize(loader)
                @loader = loader
                @tasks = Hash.new
                @deployers = Hash.new
                @define_default_deployments = true
                @enabled_transports = Set.new
            end

	    # Gets or sets the project's name
            #
            # @overload name
            #   @return [String] this project's name
            # @overload name(new_name)
            #   @param [String] the name that should be set
            #   @return [self]
	    dsl_attribute :name do |new|
                if !new.respond_to?(:to_str)
                    raise ArgumentError, 'name should be a string'
                end
		new
	    end


            # Gets or sets the project's version
            #
            # @overload version
	    #   The version number of this project. Defaults to "0.0"
            #   @return [String]
            # @overload version(new_version)
            #   @param [String] new_version the new version
            #   @return [String]
	    dsl_attribute(:version) do |name|
		name = name.to_s
		if name !~ /^\d/
		    raise ArgumentError, "version strings must start with a number (had: #{name})"
		end
		name
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
                sizes = Orocos::Spec::Port.validate_max_sizes_spec(nil, values)
                @max_sizes[type.name].merge!(sizes, &block)
            end

            def validate_max_sizes_spec
                max_sizes.dup.each do |type, sizes|
                    type = intermediate_type_for(type)
                    sizes = Orocos::Spec::Port.validate_max_sizes_spec(type, sizes)
                    max_sizes[type.name].merge!(sizes)
                end
            end

            @@standard_tasks = nil
            @@standard_task_specs = { "rtt.orogen" => OROGEN_LIB_DIR, "ocl.orogen" => OROGEN_LIB_DIR }

            def self.add_standard_task_spec(filename, directory)
                @@standard_task_specs[filename] = directory
            end

            # The set of standard project defined by RTT and OCL. They are
            # defined as orogen-specification in the <tt>rtt.orogen</tt> and
            # <tt>ocl.orogen</tt>, present in orogen source code.
            def self.standard_tasks
                if @@standard_tasks
                    @@standard_tasks
                else
                    @@standard_tasks = []
                    @@standard_task_specs.each do |orogen, dir|
                        project = ImportedProject.load(nil, nil, File.expand_path(orogen, dir))
                        project.orogen_project = false
                        @@standard_tasks.concat project.tasks.values
                    end
                end

                @@standard_tasks
            end

            # Returns the TaskContext object for the default task contexts
            # superclass (i.e. RTT::TaskContext)
            def default_task_superclass
                loader.task_model_from_name "RTT::TaskContext"
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

                name = Generation.verify_valid_identifier(name)

                task = external_task_context(name, options, &block)
                task.extended_state_support

                if !task.abstract? && define_default_deployments?
                    simple_deployment(Generation.default_deployment_name(task.name), task.name)
                end

                task
	    end

            # Declares a task context that has not been generated by oroGen
            #
            # @options options [Class] type (Orocos::Spec::TaskContext) the
            #   class of the created task context
            def external_task_context(name, options = Hash.new, &block)
                options = Kernel.validate_options options,
                    :class => Orocos::Spec::TaskContext

		new_task = options[:class].new(self, "#{self.name}::#{name}")
                Spec.load_documentation(new_task, /^task_context/)
		new_task.instance_eval(&block) if block_given?
		tasks[new_task.name] = new_task
		new_task
            end

            # True if there is a deployment with the given name in this oroGen
            # project
            def has_deployment?(name)
                deployers.has_key?(name)
            end

            def find_type(name)
                typekit.find_type(name)
            end

            def find_interface_type(name)
                typekit.find_interface_type(name)
            end

            # Returns the deployment model with the given name
            #
            # @return [Orocos::Spec::Deployment,nil] the model found, or nil if
            #   none is registered with that name
            def find_deployment_by_name(name)
                deployers[name]
            end

	    # Defines a deployment, i.e. an Unix executable in which a certain
	    # number of TaskContext are instanciated, associated with threads
	    # and triggers and (optionally) connected to each other and/or
	    # started.
	    #
            # The statements in the given block are method calls to a
            # {Spec::Deployment} instance, so see the documentation of that class
            # for more information.
	    def deployment(name, &block) # :yield:
                if has_deployment?(name)
                    raise ArgumentError, "there is already a deployment named '#{name}' in this oroGen project"
                end

                deployer = Spec::Deployment.new(self, name, &block)
                enabled_transports.each do |t|
                    deployer.enable_transport(t)
                end
                deployer.instance_eval(&block) if block_given?
                deployers[deployer.name] = deployer
                deployer
	    end

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

            # Enable the given transports
            #
            # @return [Set<String>] the set of newly enabled transports (i.e.
            #   transports that are in the arguments and were not yet enabled)
            def enable_transports(*transport_names)
                new_transports = transport_names.to_set - enabled_transports
                @enabled_transports |= new_transports
                new_transports.each do |name|
                    deployers.each do |d|
                        d.enable_transport(name)
                    end
                end
                new_transports
            end
        end
    end
end
