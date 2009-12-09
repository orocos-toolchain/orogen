require 'set'

module Orocos
    module Generation
        class GenericObjectDeployment
            attr_reader :activity, :interface_object
            def initialize(activity, interface_object)
                @activity, @interface_object = activity, interface_object
            end
            def name; interface_object.name end
        end
        class PortDeployment     < GenericObjectDeployment; end
        class PropertyDeployment < GenericObjectDeployment
            attr_reader :value
            def set(value)
                @value = value
            end
        end
        class MethodDeployment   < GenericObjectDeployment; end
        class CommandDeployment  < GenericObjectDeployment; end

        class ConnPolicy
            # The connection type. Can be either :data (the default) or :buffer
            attr_accessor :type
            # The connection lock type. Can be either :lock_free (the default) or :locked
            attr_accessor :lock_policy
            # If false (the default), samples are pushed across process
            # boundaries automatically. If true, then the accessor side has to
            # require the sample to be transferred.
            attr_accessor :pull
            # If +type+ is :buffer, the size of the buffer
            attr_accessor :size

            def initialize
                @type      = :data
                @lock_policy = :lock_free
                @pull      = false
                @size      = 0
            end

            # Creates a new ConnPolicy object from a hash. For instance,
            #
            #   ConnPolicy.to_hash :type => :data, :lock_policy => :lock_free
            #
            # Would create a connection policy with the given type and lock
            # type, using default values for the rest. See attributes of
            # ConnPolicy for valid parameters and valid values.
            def self.from_hash(hash)
                hash = hash.to_hash.dup

                policy = ConnPolicy.new
                type = hash.delete(:type) || :data
                if type == :data || type == :buffer
                    policy.type = type
                else
                    raise ArgumentError, "'type' can only be either :data or :buffer"
                end
                    
                lock_policy = hash.delete(:lock_policy) || :lock_free
                if lock_policy == :lock_free || lock_policy == :locked
                    policy.lock_policy = lock_policy
                else
                    raise ArgumentError, "'lock_policy' can only be either :lock_free or :locked"
                end

                policy.pull = !!hash.delete(:pull)

                size = hash.delete(:size)
                if type == :data && size
                    raise ArgumentError, "data connections don't have any size"
                end
                policy.size = Integer(size || 0)

                if !hash.empty?
                    raise ArgumentError, "unknown policy specification options #{hash.keys.join(", ")}"
                elsif type == :buffer && policy.size == 0
                    raise ArgumentError, "you have to specify the buffer size"
                end

                policy
            end

            def to_code(varname)
                str = "ConnPolicy #{varname};\n"
                str << "#{varname}.type      = RTT::ConnPolicy::#{type.to_s.upcase};\n"
                str << "#{varname}.lock_policy = RTT::ConnPolicy::#{lock_policy.to_s.upcase};\n"
                str << "#{varname}.init      = false;\n"
                str << "#{varname}.pull      = #{pull ? "true" : "false"};\n"
                str << "#{varname}.size      = #{size};\n"
                str
            end
        end

        # The instances of this class hold the deployment-specific information
        # needed for a task.
        class TaskDeployment
            # The task name as specified by the user
            attr_reader :name
            # The TaskContext used to define this task
            attr_reader :context

            attr_reader :properties
            attr_reader :ports
            attr_reader :methods
            attr_reader :commands

            def initialize(name, context)
                @name     = name
                @context  = context
		@realtime = false
		@priority = :lowest
                @max_overruns = 5
                if context.default_activity
                    send(*context.default_activity)
                end

                { :properties  => PropertyDeployment,
                    :ports    => PortDeployment,
                    :methods  => MethodDeployment,
                    :commands => CommandDeployment }.each do |collection_name, klass|
                        deployed_objects = context.send("all_#{collection_name}").map do |obj|
                            klass.new(self, obj)
                        end
                        instance_variable_set "@#{collection_name}", deployed_objects
                    end
            end
            
	    # Make this task as being of the highest priority allowed by the
	    # underlying OS
	    def highest_priority; @priority = :highest; self end
	    # Make this task as being of the lowest priority allowed by the
	    # underlying OS
	    def lowest_priority;  @priority = :lowest; self end
	    # call-seq:
	    #	priority prio => new_prio
            #	priority => current_priority
	    #
	    # Sets the task priority as an integer value. Allowed values are
	    # OS-specific, and for now the allowed range is unfortunately not
	    # available from Ruby.
            #
            # See also #highest_priority and #lowest_priority
	    dsl_attribute(:priority) { |value| Integer(value) }

            # The Component object this task is part of
            def component; context.component end

            # The subclass of ActivityInterface which should be used to run this task
            dsl_attribute :activity_type do |type|
                if context.required_activity? && activity_type
                    raise ArgumentError, "the #{context.name} task context requires #{activity_type} as an activity, you cannot change it"
                end
                type.to_s
            end

            # Makes this task's activity driven by a file descriptor. The underlying
            # task context must be a subclass of FileDescriptorActivity::Provider
            def fd_driven; activity_type 'FileDescriptorActivity'; self end

            # Makes this task's activity driven by events on an IRQ line. The
            # underlying task context must be a subclass of
            # IRQActivity::Provider
            def irq_driven; activity_type 'IRQActivity'; self end

            # Make this task being driven "on demand", i.e. when the step()
            # method is explicitely called on it.
            def slave; activity_type 'SlaveActivity'; self end

            # Make this task being driven by a set of events. If the task is
            # itself declared data driven (through the TaskContext#data_driven
            # call), the task will be triggered by the data availability on its
            # input ports.
            def event_driven; activity_type 'EventDrivenActivity'; self end

	    # Returns the task period, or nil if the task is not periodic. Call
            # #periodic to define a periodic task and one of the other
            # triggering methods otherwise.
            attr_reader :period

	    # call-seq:
	    #	periodic(period_in_seconds) => self
            #
            # Sets this task as being periodic. Call #period to return the
            # current task's period (or nil if the task is not periodic), and
            # one of the other triggering methods if you want a different
            # activity type.
            def periodic(value)
                @period = Float(value)
                activity_type 'PeriodicActivity'
                self
            end

            # Marks this task as being explicitely triggered (the default). To
            # make it periodic, call #period with the required period
	    def triggered; activity_type 'NonPeriodicActivity'; self end

            # Marks this task as being "sequential". Sequential tasks are
            # thread-less, and are triggered by the component that is calling
            # step() on them, or -- in the case of port-driven tasks -- by the
            # component that wrote on their read ports.
            def sequential; @activity_type = nil; self end

            # Call to make the deployer start this task when the component is
            # launched
            def start; @start = true; self end
            # True if this task should be started when the component is
            # started. Note that the deployer must honor the initial_state of
            # the underlying task context (i.e. call configure() if
            # initial_state is PreOperational)
            def start?; !!@start end

            # Do no check for overruns. Valid for periodic tasks. See
            # #max_overruns for a more precise description.
            def disable_overruns_control; @max_overruns = -1; self end

            # Sets or gets the count of allowed overruns. Only valid if the
            # task is periodic. Default: 5. Use #disable_overruns_control
            # to disable this check.
            dsl_attribute(:max_overruns) do |value|
                if !period
                    raise "max_overruns is only valid in periodic tasks"
                else
                    @period = Integer(value)
                end
            end

	    # Returns the Orocos scheduler constant name for this task's
	    # scheduler class. Call #realtime and #non_realtime to change the
	    # task scheduling class
	    def rtt_scheduler
		if @realtime then 'ORO_SCHED_RT'
		else 'ORO_SCHED_OTHER'
		end
	    end
	    # Marks this task as being part of the realtime scheduling class
	    def realtime; @realtime = true; self end
	    # Marks this task as being part of the non-realtime scheduling
	    # class (the default)
	    def non_realtime; @realtime = false; self end

	    # Returns the Orocos value for this task's priority
	    def rtt_priority
		case @priority
		when :highest
		    'RTT::OS::HighestPriority'
		when :lowest
		    'RTT::OS::LowestPriority'
		when Integer
		    @priority
		end
	    end

            def method_missing(m, *args) # :nodoc:
                name = m.to_s
                if name =~ /=$/
                    setter = true
                    name = $`
                end

                { :properties  => PropertyDeployment,
                    :ports    => PortDeployment,
                    :methods  => MethodDeployment,
                    :commands => CommandDeployment }.each do |collection_name, klass|
                    if obj = send(collection_name).find { |el| el.name == name.to_s }
                        if setter
                            if args.size != 1
                                raise ArgumentError, "wrong number of arguments for #{name}=: expected 1 got #{args.size}"
                            end
                            obj.set args[0]
                            return args[0]
                        else
                            if !args.empty?
                                raise ArgumentError, "wrong number of arguments for #{name}: expected 1 got #{args.size}"
                            end
                            return obj
                        end
                    end
                end

                return super
            end
        end

        # Instances of this class are used to define a _deployment_. The
        # deployment is the part in which the TaskContext classes are
        # instanciated and associated with specific Activity classes.
        #
        class StaticDeployment
	    # The deployment name
	    attr_reader :name
            # The underlying Component object
            attr_reader :component
            # The set of tasks that need to be deployed
            attr_reader :task_activities

	    # Do not install that deployment
	    def do_not_install; @install = false end
	    # True if this deployment should be installed
	    def install?; !!@install end

            # True if we are generating for Linux
            def linux?;     component.linux? end
            # True if we are generating for Xenomai
            def xenomai?;   component.xenomai? end

            def initialize(component, name, &block)
		@name		 = name
		@install	 = true
                @task_activities = Array.new
                @component       = component
                @file_reporters  = Hash.new
                @loggers	 = Hash.new
                @connections     = Array.new
                @tcp_reporters   = Hash.new
                @peers           = Set.new
                @corba_enabled   = nil
                @browse          = nil
                @loglevel        = nil
            end

            KNOWN_LOG_LEVELS = {
                :info => 'Info',
                :debug => 'Debug'
            }

            # Returns the set of toolkits required by this particular
            # deployment (i.e. the tasks that are deployed in it)
            def used_toolkits
                task_toolkits = task_activities.map do |deployed_task|
                    deployed_task.context.used_toolkits.
                        map(&:name)
                end.flatten.to_set

                task_toolkits.map do |used_name|
                    this_tk = component.used_toolkits.find { |tk| tk.name == used_name }
                    if !this_tk
                        raise "Internal Error: imported toolkit is not present in this component's used_toolkits set"
                    end
                    this_tk
                end
            end

            # call-seq:
            #   loglevel level => self
            #
            # Sets the default log level. The known log levels are listed in
            # KNOWN_LOG_LEVELS
            dsl_attribute :loglevel do |level|
                orocos_level = KNOWN_LOG_LEVELS[level.to_sym]
                if !orocos_level
                    raise ArgumentError, "unknown log level '#{level}'. Must be one of #{KNOWN_LOG_LEVELS.keys.join(", ")}"
                end
                @loglevel = orocos_level
            end

            # Makes the deployed component offer a CORBA interface. By default,
            # follows the value of component.corba_enabled?, but this can be
            # overriden by using #enable_corba and #disable_corba
            def corba_enabled?; @corba_enabled.nil? ? component.corba_enabled? : @corba_enabled end

            # Force the use of corba -- even though corba is not enabled in the
            # component itself.
            def enable_corba;  @corba_enabled = true end
            # Force the non-use of corba -- even though corba is enabled in the
            # component itself.
            def disable_corba; @corba_enabled = false end

            # call-seq:
            #   task name, task_context => task_deployment
            #
            # Deploys a new task using the given task context type, and returns
            # the corresponding TaskDeployment object. This instance can be used
            # to configure the task further (for instance specifying the
            # activity). See TaskDeployment documentation for available options.
            #
            # +name+ is used in the task browser, and will be the global task's
            # name on the CORBA name server.
            def task(name, klass)
                unless task_context = component.find_task_context(klass)
                    raise "no such task context: #{klass}"
                end

                if name !~ /^[a-zA-Z_]+$/
                    raise ArgumentError, "task names can only contain alphanumeric characters and '_'"
                end

                deployment = TaskDeployment.new(name, task_context)
                task_activities << deployment
                deployment
            end

            # Generates the code associated with this deployment setup
            def generate
                deployer = self

		main = Generation.render_template 'main.cpp', binding
		Generation.save_automatic "main-#{name}.cpp", main
                pkg = if install?
                          Generation.render_template 'deployment.pc', binding
                      else
                          Generation.render_template 'local_deployment.pc', binding
                      end
                Generation.save_automatic "#{name}.pc.in", pkg
                cmake = Generation.render_template 'config/Deployment.cmake', binding
                Generation.save_automatic "config/#{name}Deployment.cmake", cmake
            end

            dsl_attribute :main_task do |task|
                @main_task = task
            end

            class Logger
                attr_reader :component, :task, :config
                def initialize(component, task)
                    @component, @task, @config = component, task, Array.new
                end

                def report(object)
                    method = case object
                             when TaskDeployment then ["Component", object, [object.name]]
                             when PortDeployment then ["Port",     object.activity, [object.activity.name, object.name]]
                             when PropertyDeployment then ["Data", object.activity, [object.activity.name, object.name]]
                             else raise ArgumentError, "cannot report #{object}"
                             end

                    task = if object.kind_of?(TaskDeployment)
                               object
                           else object.activity
                           end
                    component.add_peers self.task, task

                    config << method
                    self
                end

                def method_missing(*args)
                    task.send(*args)
                end
            end

            # Common code for reporting definitions
            def logger_for(collection, key) # :nodoc:
                collection[key] ||= Logger.new(self, yield)
            end

            attr_reader :loggers

            def add_default_logger
                task = logger(nil, "#{name}_Logger")
                task_activities.each do |act|
                    add_peers task, act
                end
                task
            end

            # Gets a data logger object, defined from the logger component, for
            # the given file
            def logger(file_name = nil, task_name = nil)
                file_name ||= name + '.log'
                file_name = file_name.to_str

                if !component.used_task_libraries.find { |t| t.name == "logger" }
                    component.using_task_library "logger"
                end

                logger_for(loggers, file_name) do
                    task_name ||= "#{name}_DataLogger#{loggers.size}"
                    task_name = task_name.to_str

                    logger = task(task_name, "logger::Logger")
                    logger.file = file_name
                    logger
                end
            end

            # The set of peer pairs set up for this deployment. This is a set
            # of [a, b] TaskDeployment objects.
            attr_reader :peers

            # The set of connections set up for this deployment. This is a set
            # of [from, to] PortDeployment objects.
            attr_reader :connections

            # Connects the two given ports or tasks
            def connect(from, to, policy = Hash.new)
                add_peers from.activity, to.activity

                if from.kind_of?(Port)
                    if !from.kind_of?(OutputPort)
                        raise ArgumentError, "in connect(a, b), 'a' must be a writer port"
                    elsif !to.kind_of?(InputPort)
                        raise ArgumentError, "in connect(a, b), 'b' must be a reader port"
                    end
                end

                connections << [from, to, ConnPolicy.from_hash(policy)]
                self
            end

            # Declare that the given tasks are peers
            def add_peers(a, b)
                peers << [a, b]
                self
            end

            # call-seq:
            #   browse => currently_browsed_task
            #   browse(task) => self
            #
            # Sets up a TaskBrowser component to browse the given task, which
            # is started when all tasks have been initialized. This is incompatible
            # with the use of CORBA and only one browser can be defined.
            dsl_attribute :browse do |task|
                if browse
                    raise ArgumentError, "can browse only one task"
                elsif corba_enabled?
                    raise ArgumentError, "cannot browse and use CORBA at the same time"
                end
                @browse = task
            end

            def used_task_libraries
                task_models = task_activities.map { |task| task.context }
                component.used_task_libraries.find_all do |tasklib|
                    puts tasklib.name
                    puts tasklib.self_tasks.map(&:name)
                    result = task_models.any? do |task|
                        tasklib.self_tasks.include?(task)
                    end
                    puts result
                    result
                end
            end

            def dependencies
                result = []
                result << BuildDependency.new("OrocosRTT", "orocos-rtt-#{Generation.orocos_target}", false, true, true)
                if browse
                    result << BuildDependency.new("OrocosOCL", "orocos-ocl-#{Generation.orocos_target}", false, true, true)
                end
                if corba_enabled?
                    result << BuildDependency.new("OrocosCORBA", "orocos-transport-corba-#{Generation.orocos_target}", true, true, true)
                end

                used_toolkits.each do |tk|
                    result << BuildDependency.new("#{tk.name}_TOOLKIT", tk.pkg_name, false, true, true)
                    if corba_enabled?
                        result << BuildDependency.new("#{tk.name}_TRANSPORT_CORBA", tk.pkg_corba_name, true, true, true)
                    end
                end

                used_task_libraries.each do |pkg|
                    result << BuildDependency.new("#{pkg.name}_TASKLIB", "#{pkg.name}-tasks-#{Generation.orocos_target}", false, true, true)
                end

                # Task files could be using headers from external libraries, so add the relevant
                # directory in our include path
                component.tasklib_dependencies.
                    find_all { |builddep| !builddep.corba && builddep.include }.
                    each do |builddep|
                        builddep.link = false
                        result << builddep
                    end

                result.to_a.sort_by { |dep| dep.var_name }
            end
        end
    end
end


