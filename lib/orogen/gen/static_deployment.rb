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
        class OperationDeployment   < GenericObjectDeployment; end

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
            #   ConnPolicy.to_hash :type -> :data, :lock_policy -> :lock_free
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
                str = "RTT::ConnPolicy #{varname};\n"
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
            # The TaskContext model used to define this task
            attr_reader :task_model
            # Backward compatibility only. Use #task_model instead.
            def context; task_model end

            # The deployed properties, as PropertyDeployment instances
            attr_reader :properties
            # The deployed ports, as PortDeployment instances
            attr_reader :ports
            # The deployed operations, as OperationDeployment instances
            attr_reader :operations

            # Overrides the default minimal trigger latency for this particular
            # task
            #
            # See minimal_trigger_latency
            attr_accessor :minimal_trigger_latency
            # Overrides the default expected trigger latency for this particular
            # task
            #
            # See worstcase_trigger_latency
            attr_accessor :worstcase_trigger_latency

            # Default minimal latency value used for realtime scheduling
            #
            # See minimal_trigger_latency
            DEFAULT_RT_MINIMAL_TRIGGER_LATENCY = 0.001
            # Default expected latency value used for realtime scheduling
            #
            # See worstcase_trigger_latency
            DEFAULT_RT_WORSTCASE_TRIGGER_LATENCY = 0.005
            # Default minimal latency value used for non-realtime scheduling
            #
            # See minimal_trigger_latency
            DEFAULT_NONRT_MINIMAL_TRIGGER_LATENCY = 0.005
            # Default expected latency value used for non-realtime scheduling
            #
            # See worstcase_trigger_latency
            DEFAULT_NONRT_WORSTCASE_TRIGGER_LATENCY = 0.020

            # Returns the minimal latency between the time the task gets
            # triggered (for instance because of data on an input event port),
            # and the time updateHook() is actually called, based on its
            # scheduler and priority. All tasks will return a value (even
            # non-periodic ones).
            #
            # Default values are set in the DEFAULT_RT_MINIMAL_TRIGGER_LATENCY
            # and DEFAULT_NONRT_MINIMAL_TRIGGER_LATENCY constants. They can be
            # overriden by setting the minimal_trigger_latency property
            def minimal_trigger_latency
                if @minimal_trigger_latency
                    @minimal_trigger_latency
                elsif @realtime
                    DEFAULT_RT_MINIMAL_TRIGGER_LATENCY
                else
                    DEFAULT_NONRT_MINIMAL_TRIGGER_LATENCY
                end
            end

            # Returns the expected (average) latency between the time the task
            # gets triggered (for instance because of data on an input event
            # port), and the time updateHook() is actually called, based on its
            # scheduler and priority. All tasks will return a value (even
            # non-periodic ones).
            #
            # Default values are set in the DEFAULT_RT_WORSTCASE_TRIGGER_LATENCY
            # and DEFAULT_NONRT_WORSTCASE_TRIGGER_LATENCY constants. They can be
            # overriden by setting the worstcase_trigger_latency property
            def worstcase_trigger_latency
                computation_time = task_model.worstcase_processing_time || 0

                trigger_latency =
                    if @worstcase_trigger_latency
                        @worstcase_trigger_latency
                    elsif @realtime
                        DEFAULT_RT_WORSTCASE_TRIGGER_LATENCY
                    else
                        DEFAULT_NONRT_WORSTCASE_TRIGGER_LATENCY
                    end
                [computation_time, trigger_latency].max
            end

            def initialize(name, task_model)
                @name     = name
                @task_model  = task_model
		@realtime = false
		@priority = :lowest
                @max_overruns = -1
                if task_model.default_activity
                    send(*task_model.default_activity)
                    @explicit_activity = task_model.required_activity?
                end

                { :properties  => PropertyDeployment,
                    :ports    => PortDeployment,
                    :operations => OperationDeployment }.each do |collection_name, klass|
                        deployed_objects = task_model.send("all_#{collection_name}").map do |obj|
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
	    #	priority prio -> new_prio
            #	priority -> current_priority
	    #
	    # Sets the task priority as an integer value. Allowed values are
	    # OS-specific, and for now the allowed range is unfortunately not
	    # available from Ruby.
            #
            # See also #highest_priority and #lowest_priority
	    dsl_attribute(:priority) { |value| Integer(value) }

            # The Project object this task is part of
            def project; task_model.project end
            # Backward compatibility only. Use #project instead
            def component; project end

            ActivityDefinition = Struct.new :name, :class_name, :header

            # The subclass of ActivityInterface which should be used to run this task
            dsl_attribute :activity_type do |*type|
                if @explicit_activity
                    if task_model.required_activity?
                        raise ArgumentError, "the #{task_model.name} task context requires #{@activity_type.name} as an activity, you cannot change it"
                    else
                        raise ArgumentError, "you already explicitely set the activity of #{name}"
                    end
                end
                @explicit_activity = true
                @period = nil
                @activity_setup = nil
                
                ActivityDefinition.new(type[0], type[1], type[2])
            end

            # Makes this task's activity driven by a file descriptor. The underlying
            # task context must be a subclass of FileDescriptorActivity::Provider
            def fd_driven
                activity_type 'FileDescriptorActivity', 'RTT::extras::FileDescriptorActivity', 'rtt/extras/FileDescriptorActivity.hpp'
                activity_xml do
                    result = <<-EOD
<struct name="#{name}" type="FileDescriptorActivity">
    <simple name="Priority" type="short"><value>#{rtt_priority}</value></simple>
    <simple name="Scheduler" type="string"><value>#{rtt_scheduler}</value></simple>
</struct>
                    EOD
                end
                self
            end

            # Makes this task's activity driven by events on an IRQ line. The
            # underlying task context must be a subclass of
            # IRQActivity::Provider
            def irq_driven
                activity_type 'IRQActivity', 'RTT::extras::IRQActivity', 'rtt/extras/IRQActivity.hpp'
                activity_xml do
                    result = <<-EOD
<struct name="#{name}" type="IRQActivity">
    <simple name="Priority" type="short"><value>#{rtt_priority}</value></simple>
    <simple name="Scheduler" type="string"><value>#{rtt_scheduler}</value></simple>
</struct>
                    EOD
                end
                self
            end

            # Make this task being driven "on demand", i.e. when the step()
            # method is explicitely called on it.
            def slave
                activity_type 'SlaveActivity', 'RTT::extras::SlaveActivity', 'rtt/extras/SlaveActivity.hpp'
                activity_setup do
                    result = <<-EOD
#{activity_type.class_name}* activity_#{name} = new #{activity_type.class_name}(task_#{name}.engine());
                    EOD
                end
                activity_xml do
                    "<struct name=\"#{name}\" type=\"SlaveActivity\" />"
                end
                self
            end

	    # Returns the task period, or nil if the task is not periodic. Call
            # #periodic to define a periodic task and one of the other
            # triggering methods otherwise.
            attr_reader :period

            # Returns true if +port+ is a trigger for this task
            def trigger_port?(port)
                port.trigger_port? &&
                    activity_type.name != 'Periodic'
            end

	    # call-seq:
	    #	periodic(period_in_seconds) -> self
            #
            # Sets this task as being periodic. Call #period to return the
            # current task's period (or nil if the task is not periodic), and
            # one of the other triggering methods if you want a different
            # activity type.
            def periodic(value)
                activity_type 'Periodic', 'RTT::Activity', 'rtt/Activity.hpp'
                activity_setup do
                   result = <<-EOD
#{activity_type.class_name}* activity_#{name} = new #{activity_type.class_name}(
    #{rtt_scheduler},
    #{rtt_priority},
    #{period},
    task_#{name}.engine(),
    "#{name}");
RTT::os::Thread* thread_#{name} =
    dynamic_cast<RTT::os::Thread*>(activity_#{name}->thread());
thread_#{name}->setMaxOverrun(#{max_overruns});
                   EOD
                end
                activity_xml do
                    result = <<-EOD
<struct name="#{name}" type="Activity">
    <simple name="Period" type="double"><value>#{period}</value></simple>
    <simple name="Priority" type="short"><value>#{rtt_priority}</value></simple>
    <simple name="Scheduler" type="string"><value>#{rtt_scheduler}</value></simple>
</struct>
                    EOD
                end

                @period = Float(value)
                self
            end

            # Marks this task as being explicitely triggered (the default). To
            # make it periodic, call #period with the required period
	    def triggered
                activity_type 'Triggered', 'RTT::Activity', 'rtt/Activity.hpp'
                activity_setup do
                   result = <<-EOD
#{activity_type.class_name}* activity_#{name} = new #{activity_type.class_name}(
    #{rtt_scheduler},
    #{rtt_priority},
    0,
    task_#{name}.engine(),
    "#{name}");
                   EOD
                end
                activity_xml do
                    result = <<-EOD
<struct name="#{name}" type="Activity">
    <simple name="Period" type="double"><value>0</value></simple>
    <simple name="Priority" type="short"><value>#{rtt_priority}</value></simple>
    <simple name="Scheduler" type="string"><value>#{rtt_scheduler}</value></simple>
</struct>
                    EOD
                end
                @period = 0
                self
            end

            # Marks this task as being "sequential". Sequential tasks are
            # thread-less, and are triggered by the component that is calling
            # step() on them, or -- in the case of port-driven tasks -- by the
            # component that wrote on their read ports.
            def sequential
                activity_type 'Sequential', 'RTT::extras::SequentialActivity', 'rtt/extras/SequentialActivity.hpp'
                activity_setup do
                    result = <<-EOD
#{activity_type.class_name}* activity_#{name} = new #{activity_type.class_name}(task_#{name}.engine());
                    EOD
                end
                activity_xml do
                    result = <<-EOD
<struct name="#{name}" type="SequentialActivity" />
                    EOD
                end

                self
            end

            def activity_setup(&block)
                @activity_setup = block
            end

            def activity_xml(&block)
                @activity_xml = block
            end

            def generate_activity_setup
                if @activity_setup
                    @activity_setup.call
                else
                    result = <<-EOD
#{activity_type.class_name}* activity_#{name} = new #{activity_type.class_name}(
            #{rtt_scheduler},
            #{rtt_priority},
            task_#{name}.engine(),
            "#{name}");
                    EOD
                end
            end

            def to_deployer_xml
                @activity_xml.call
            end

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

            # True if this task should be deployed using a realtime scheduler,
            # and false otherwise
            def realtime?; @realtime end

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
		    'RTT::os::HighestPriority'
		when :lowest
		    'RTT::os::LowestPriority'
		when Integer
		    @priority
		end
	    end

            def pretty_print(pp) # :nodoc:
                pp.text "#{name}[#{task_model.name}]"
                pp.nest(2) do
                    pp.breakable
                    pp.text "activity: #{@activity_type.name}, prio=#{@priority}"
                    pp.breakable
                    pp.text "scheduler: #{@realtime ? 'realtime' : 'non realtime'}"
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
                    :operations  => OperationDeployment }.each do |collection_name, klass|
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
        class Deployment
	    # The deployment name
	    attr_reader :name
            # The underlying Project object
            attr_reader :project
            # Backward compatibility only
            def component; project end
            # The set of tasks that need to be deployed
            attr_reader :task_activities

						#Array for additional Loaded Types

	    # Do not install that deployment
	    def do_not_install; @install = false end
	    # True if this deployment should be installed
	    def install?; !!@install end

            # True if we are generating for Linux
            def linux?;     project.linux? end
            # True if we are generating for Xenomai
            def xenomai?;   project.xenomai? end

            def initialize(project, name)
		@name		 = name
		@install	 = true
                @task_activities = Array.new
                @project       = project
                @file_reporters  = Hash.new
                @loggers	 = Hash.new
                @connections     = Array.new
                @tcp_reporters   = Hash.new
                @peers           = Set.new
                @corba_enabled   = nil
                @browse          = nil
                @loglevel        = nil
                @transports      = Array.new
                @manually_loaded_types = Set.new
            end

            KNOWN_LOG_LEVELS = {
                :info => 'Info',
                :debug => 'Debug'
            }

            #Manually Request Loading of Types that are not Provided throught the task_contex typekit.
            def load_type(typename)
                if !project.imported_typekits_for(type).map(&:name)[0]
                    raise ArgumentError, "cannot find a typekit defining #{typename}"
                end
                @manually_loaded_types << typename
            end

            # Returns the set of typekits required by this particular
            # deployment (i.e. the tasks that are deployed in it)
            def used_typekits
                task_typekits = task_activities.map do |deployed_task|
                    deployed_task.task_model.used_typekits.
                        map(&:name)
                end.flatten.to_set
               
                @manually_loaded_types.each do |type|
                    tk = project.imported_typekits_for(type).map(&:name)[0]
                    if !tk
                        raise InternalError, "could not find manually loaded type \"#{type}\""
                    end
                    task_typekits << tk
                end

                task_typekits.sort.map do |used_name|
                    this_tk = project.find_typekit(used_name)
                    next if this_tk.virtual?
                    if !this_tk
                        raise InternalError, "#{used_name} is a typekit that is listed by one of the tasks of the #{name} deployment, but the oroGen project #{project.name} does not list it"
                    end
                    this_tk
                end.compact
            end

            # call-seq:
            #   loglevel level -> self
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

            # The set of transports loaded by this deployment for which a
            # transport should be loaded on the RTT itself
            def rtt_transports
                result = self.transports
                result.delete('typelib')
                result
            end
            # The set of transports loaded by this deployment, as an array of
            # names
            def transports
                @transports.to_a.sort
            end
            # Forbid the deployment from loading the given transport
            def disable_transport(transport_name)
                @transports.delete(transport_name.to_str)
            end
            # Make the deployment load the given transport
            def enable_transport(transport_name)
                @transports << transport_name.to_str
            end

            # call-seq:
            #   task name, task_context -> task_deployment
            #
            # Deploys a new task using the given task context type, and returns
            # the corresponding TaskDeployment object. This instance can be used
            # to configure the task further (for instance specifying the
            # activity). See TaskDeployment documentation for available options.
            #
            # +name+ is used in the task browser, and will be the global task's
            # name on the CORBA name server.
            def task(name, klass)
                begin task_context = project.find_task_context(klass)
                rescue ArgumentError
                    raise ConfigError, "#{klass} is not a known task context model"
                end

                name = Generation.verify_valid_identifier(name)
                deployment = TaskDeployment.new(name, task_context)
                task_activities << deployment
                deployment
            end

            # True if this deployment should export its tasks through CORBA.
            #
            # It is true by default if the CORBA transport is enabled
            def corba_enabled?
                if @corba_enabled.nil?
                    transports.include?('corba')
                else @corba_enabled
                end
            end

            # Force disabling CORBA support even though the CORBA transport is
            # enabled in this deployment
            #
            # See #corba_enabled?
            def disable_corba; @corba_enabled = false end

            # Force enabling CORBA support even though the CORBA transport is
            # not enabled in this deployment
            #
            # See #corba_enabled?
            def enable_corba; @corba_enabled = true end

            # Generates the code associated with this deployment setup
            def generate
                deployer = self

                if !corba_enabled? && !@browse
                    STDERR.puts "WARN: the deployment #{name} will do nothing. Either generate with --transports=corba or use the 'browse' statement"
                end

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

                if !project.used_task_libraries.find { |t| t.name == "logger" }
                    project.using_task_library "logger"
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
            #   browse -> currently_browsed_task
            #   browse(task) -> self
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
                task_models = Set.new
                task_activities.each do |task|
                    task_models |= task.task_model.ancestors.to_set
                end
                task_models.delete_if do |task|
                    !task.project.orogen_project?
                end

                dependencies = project.used_task_libraries.find_all do |tasklib|
                    current_size = task_models.size
                    tasklib_tasks = tasklib.self_tasks
                    task_models.delete_if do |task|
                        tasklib_tasks.include?(task)
                    end
                    current_size != task_models.size
                end

                if !task_models.all? { |t| project.self_tasks.include?(t) }
                    raise ArgumentError, "cannot find an imported task library which defines #{task_models.map(&:name).join(", ")}"
                end
                dependencies
            end

            def dependencies
                result = []
                result << BuildDependency.new(
                    "OrocosRTT",
                    "orocos-rtt-#{Generation.orocos_target}").
                    in_context('core', 'include').
                    in_context('core', 'link')

                if browse
                    result << BuildDependency.new(
                        "OrocosOCL",
                        "orocos-ocl-#{Generation.orocos_target}").
                        in_context('core', 'include').
                        in_context('core', 'link')
                end
                if corba_enabled?
                    result << BuildDependency.new(
                        "OrocosCORBA",
                        "orocos-rtt-corba-#{Generation.orocos_target}").
                        in_context('corba', 'include').
                        in_context('corba', 'link')
                end

                used_typekits.each do |tk|
                    next if tk.virtual?
                    result << BuildDependency.new(
                        "#{tk.name}_TYPEKIT",
                        tk.pkg_name).
                        in_context('core', 'include').
                        in_context('core', 'link')

                    transports.each do |transport_name|
                        result << BuildDependency.new(
                            "#{tk.name}_TRANSPORT_#{transport_name.upcase}",
                            tk.pkg_transport_name(transport_name)).
                            in_context('core', 'include').
                            in_context('core', 'link')
                    end
                end

                project.used_libraries.each do |pkg|
                    result << BuildDependency.new(
                        "#{pkg.name}",
                        "#{pkg.name}").
                        in_context('core', 'include').
                        in_context('core', 'link')
                end

                used_task_libraries.each do |pkg|
                    result << BuildDependency.new(
                        "#{pkg.name}_TASKLIB",
                        "#{pkg.name}-tasks-#{Generation.orocos_target}").
                        in_context('core', 'include').
                        in_context('core', 'link')
                end

                # Task files could be using headers from external libraries, so add the relevant
                # directory in our include path
                project.tasklib_dependencies.
                    find_all { |builddep| builddep.in_context?('core', 'include') }.
                    each do |builddep|
                        builddep = BuildDependency.new(builddep.var_name, builddep.pkg_name)
                        builddep.in_context('core', 'include')
                        result << builddep
                    end

                result.to_a.sort_by { |dep| dep.var_name }
            end

            def to_deployer_xml
                result = []
                result << <<-EOHEADER
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE properties SYSTEM "cpf.dtd">
<properties>
                EOHEADER
                used_typekits.each do |tk|
                    next if tk.virtual?
                    result << "<simple name=\"Import\" type=\"string\"><value>#{tk.name}</value></simple>"
                end
                used_task_libraries.each do |pkg|
                    result << "<simple name=\"Import\" type=\"string\"><value>#{pkg.name}</value></simple>"
                end

                task_activities.each do |task|
                    result << task.to_deployer_xml
                end
                result << "</properties>"
                result.join("\n")
            end

            # Displays this deployment's definition nicely
            def pretty_print(pp) # :nodoc:
                pp.text "------- #{name} ------"
                pp.breakable
                if !task_activities.empty?
                    pp.text "Tasks"
                    pp.nest(2) do
                        pp.breakable
                        pp.seplist(task_activities) do |act|
                            act.pretty_print(pp)
                        end
                    end
                end

                if !connections.empty?
                    pp.breakable if !task_activities.empty?
                    pp.text "Connections"
                    pp.nest(2) do
                        pp.breakable
                        pp.seplist(connections) do |conn|
                            from, to, policy = *conn
                            pp.text "#{from.activity.name} => #{to.activity.name} [#{policy.inspect}]"
                        end
                    end
                end
            end
        end

        StaticDeployment = Deployment
    end
end

