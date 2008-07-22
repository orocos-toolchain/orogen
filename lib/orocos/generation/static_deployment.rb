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

        class TaskDeployment
            attr_reader :name
            attr_reader :context
            attr_reader :realtime
            attr_reader :priority

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

                { :properties  => PropertyDeployment,
                    :ports    => PortDeployment,
                    :methods  => MethodDeployment,
                    :commands => CommandDeployment }.each do |collection_name, klass|
                        deployed_objects = context.send(collection_name).map do |obj|
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
	    #
	    # Sets the task priority as an integer value. Allowed values are
	    # OS-specific, and for now the allowed range is unfortunately not
	    # available from Ruby
	    dsl_attribute(:priority) { |value| Integer(value) }

            def component; context.component end

            # The subclass of ActivityInterface which should be used to run this task
            attr_reader :activity_type

            # Makes this task's activity driven by a file descriptor. The underlying
            # task context must be a subclass of FileDescriptorActivity::Provider
            def fd_driven; @activity_type = 'FileDescriptorActivity'; self end

            # Makes this task's activity driven by events on an IRQ line. The
            # underlying task context must be a subclass of
            # IRQActivity::Provider
            def irq_driven; @activity_type = 'IRQActivity'; self end

	    # call-seq:
	    #	period(period_in_seconds) => period_in_seconds
	    #
	    # Sets or gets the task period. Call #aperiodic to make it
	    # aperiodic
	    dsl_attribute(:period) do |value|
                value = Float(value)
                @activity_type = 'PeriodicActivity'
                value
            end

	    # Marks this task as being aperiodic (the default). To make it
	    # periodic, call #period with the required period
	    def aperiodic; @activity_type = 'NonPeriodicActivity'; self end

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
	    # class
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

        class StaticDeployment
            # The underlying Component object
            attr_reader :component
            # The set of tasks that need to be deployed
            attr_reader :task_activities

            # True if we are generating for Linux
            def linux?;     component.linux? end
            # True if we are generating for Xenomai
            def xenomai?;   component.xenomai? end

            def initialize(component, &block)
                @task_activities = Array.new
                @component = component
                @file_reporters = Hash.new
                @connections = Array.new
                @tcp_reporters = Hash.new
                @peers       = Set.new
            end

            KNOWN_LOG_LEVELS = {
                :info => 'Info',
                :debug => 'Debug'
            }

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
            def enable_corba;  @corba_enabled = true end
            def disable_corba; @corba_enabled = false end

            # Deploys a new task using the given context name, and returns the
            # corresponding TaskDeployment object.
            def task(name, context = name)
                name    = name.to_s
                context = context.to_s

                task_context = component.tasks.find { |t| t.name == context }
                if !task_context
                    raise ArgumentError, "no such task context #{context} defined"
                end

                deployment = TaskDeployment.new(name, task_context)
                task_activities << deployment

                if task_context.default_activity
                    deployment.send(*task_context.default_activity)
                end
                deployment
            end

            # Generates the code associated with this deployment setup
            def generate
                deployer = self

		# Generate the main.cpp file, which includes the ORO_main entry
		# point
		main = Generation.render_template 'main.cpp', binding
		Generation.save_automatic 'main.cpp', main
            end

            def cmake_code # :nodoc:
                deployer = self

                Generation.render_template 'config/static_deployer.cmake', binding
            end

            dsl_attribute :main_task do |task|
                @main_task = task
            end

            # The set of file reporters, as filename => [reporter_task, method_specifications]
            attr_reader :file_reporters

            # Common code for reporting definitions
            def reporter(collection, key, object, peek) # :nodoc:
                unless reporter_config = collection[key]
                    reporter = yield
                    reporter_config = collection[key] =  [reporter, []]
                end

                method = case object
                         when TaskDeployment then ["Component", peek, object, [object.name]]
                         when PortDeployment then ["Port",     peek, object.activity, [object.activity.name, object.name]]
                         when PropertyDeployment then ["Data", peek, object.activity, [object.activity.name, object.name]]
                         else raise ArgumentError, "cannot report #{object}"
                         end

                reporter_config[1] << method
                reporter_config[0]
            end

            # Sets up a file reporter for the given object (property, port or
            # task context)
            def file_report(object, filename, peek = true)
                filename = filename.to_s
                if !component.tasks.find { |t| t.name == "OCL::FileReporting" }
                    # Define the FileReporting task context
                    component.task_context 'FileReporting' do |task|
                        task.instance_variable_set :@name, "OCL::FileReporting"
                        task.external_definition = true
                        property 'ReportFile', 'std/string'
                    end
                end

                reporter(file_reporters, filename, object, peek) do
                    reporter = task "FileReporter#{file_reporters.size}", "OCL::FileReporting"
                    reporter.ReportFile = filename
                    reporter
                end
            end
            
            # The set of tcp reporters, as port => [reporter_task, method_specifications]
            attr_reader :tcp_reporters

            # Sets up a tcp reporter for the given object (property, port or
            # task context) on the given port
            def tcp_report(object, port, peek)
                port = Integer(port)
                if tcp_reporters.empty?
                    # Define the FileReporting task context
                    component.task_context 'TCPReporting' do |task|
                        task.instance_variable_set :@name, "OCL::TCPReporting"
                        task.external_definition = true
                        property 'port', 'int', 3142
                    end
                end

                reporter(tcp_reporters, port, object, peek) do
                    reporter = task("TCPReporter#{tcp_reporters.size}", "OCL::TCPReporting")
                    reporter.port = port
                    reporter
                end
            end

            # The set of peer pairs set up for this deployment. This is a set
            # of [a, b] TaskDeployment objects.
            attr_reader :peers

            # The set of connections set up for this deployment. This is a set
            # of [from, to] PortDeployment objects.
            attr_reader :connections

            # Connects the two given ports
            def connect(from, to)
                add_peers from.activity, to.activity
                connections << [from, to]
            end

            def add_peers(a, b)
                peers << [a, b]
            end

            # Sets up the TaskBrowser component and uses it to browse
            # +task_name+
            dsl_attribute :browse do |task|
                if browse
                    raise ArgumentError, "can browse only one task"
                elsif corba_enabled?
                    raise ArgumentError, "cannot browse and use CORBA at the same time"
                end
                @browse = task
            end
        end
    end
end


