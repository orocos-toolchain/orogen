module OroGen
    module Spec
        # Representation of an oroGen project
        class Project
            def self.default_deployment_name(task_model_name)
                "orogen_default_#{task_model_name.gsub(/[^\w]/, '_')}"
            end

            # The loader that should be used to get our dependencies
            attr_reader :loader
            # This project's typekit
            # @return [Typekit,nil]
            attr_writer :typekit
            def typekit(create = nil)
                @typekit
            end
            # The tasks known to this project
            # @return [Hash<String,OroGen::Spec::TaskContext>]
            attr_reader :tasks
            # The tasks defined by this project
            # @return [Hash<String,OroGen::Spec::TaskContext>]
            attr_reader :self_tasks
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
                @self_tasks = Hash.new
                @deployers = Hash.new
                @define_default_deployments = true
                @enabled_transports = Set.new
                @max_sizes = Hash.new
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
                sizes = Port.validate_max_sizes_spec(nil, values)
                @max_sizes[type.name] ||= Hash.new
                @max_sizes[type.name].merge!(sizes, &block)
            end

            def validate_max_sizes_spec
                @max_sizes.dup.each do |type, sizes|
                    type = intermediate_type_for(type)
                    sizes = Port.validate_max_sizes_spec(type, sizes)
                    @max_sizes[type.name] ||= Hash.new
                    @max_sizes[type.name].merge!(sizes)
                end
            end

            # Makes the tasks defined by the given task library known to this
            # project
            #
            # @param [String,Project] tasklib the task library, or its name
            # @return [Project] the task library
            def using_task_library(tasklib)
                if tasklib.respond_to?(:to_str)
                    tasklib = loader.task_library_model_from_name(tasklib)
                end
                tasks.merge! tasklib.self_tasks
                tasklib
            end

            attr_writer :default_task_superclass

            # Returns the TaskContext object for the default task contexts
            # superclass (i.e. RTT::TaskContext)
            def default_task_superclass
                if @default_task_superclass.nil?
                    @default_task_superclass = loader.task_model_from_name "RTT::TaskContext"
                else @default_task_superclass
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

                task = external_task_context(name, options, &block)
                task.extended_state_support
                self_tasks[task.name] = task

                if !task.abstract? && define_default_deployments?
                    simple_deployment(Project.default_deployment_name(task.name), task.name)
                end

                task
	    end

            # Declares a task context that has not been generated by oroGen
            #
            # @options options [Class] type ({TaskContext}) the
            #   class of the created task context
            def external_task_context(name, options = Hash.new, &block)
                options = Kernel.validate_options options,
                    :class => TaskContext

		new_task = options[:class].new(self, "#{self.name}::#{name}")
		new_task.instance_eval(&block) if block_given?
		tasks[new_task.name] = new_task
                loader.loaded_task_models[new_task.name] = new_task
		new_task
            end

            def using_typekit(typekit)
                if typekit.respond_to?(:to_str)
                    loader.typekit_model_from_name(typekit)
                else loader.register_typekit_objects(typekit)
                end
            end

            def import_types_from(typekit)
                if typekit.respond_to?(:to_str) && !loader.has_typekit?(typekit)
                    return
                end
                using_typekit(typekit)
            end

            # (see Loaders::Base#task_model_from_name)
            def task_model_from_name(name)
                if name !~ /::/
                    name = "#{self.name}::#{name}"
                end
                tasks[name] || loader.task_model_from_name(name)
            end

            # @deprecated use {task_model_from_name} instead
            def find_task_context(name)
                task_model_from_name(name)
            rescue OroGen::TaskModelNotFound => e
                raise ArgumentError, e.message, e.backtrace
            end

            # True if there is a deployment with the given name in this oroGen
            # project
            def has_deployment?(name)
                deployers.has_key?(name)
            end

            def resolve_type(type)
                loader.resolve_type(type)
            end

            def resolve_interface_type(type)
                loader.resolve_interface_type(type)
            end

            def find_type(name)
                loader.resolve_type(name)
            end

            def find_interface_type(name)
                loader.resolve_interface_type(name)
            end

            def intermediate_type_for(type)
                loader.intermediate_type_for(type)
            end

            # Returns the deployment model with the given name
            #
            # @return [Deployment,nil] the model found, or nil if
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
                has_logger = loader.has_project?('logger')
                if has_logger
                    using_task_library "logger"
                end

                result = nil
                deployment name do
                    result = task name, klass
                    if has_logger
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
                        pp.seplist(self_tasks.values.sort_by(&:name)) do |t|
                            t.pretty_print(pp)
                        end
                    end
                end

                if !deployers.empty?
                    pp.breakable if !self_tasks.empty?
                    pp.text "  Deployers:"
                    pp.nest(4) do
                        pp.breakable
                        pp.seplist(deployers.values.sort_by(&:name)) do |d|
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

            # Enumerate this project's deployments
            #
            # @yieldparam [Deployment] deployment
            # @return [void]
            def each_deployment
                return enum_for(__method__) if !block_given?
                deployers.each_value(&proc)
            end
        end
    end
end
