module Orocos
    module Generation
        class TaskDeployment
            attr_reader :name
            attr_reader :context
            attr_reader :realtime
            attr_reader :priority

            def initialize(name, context)
                @name     = name
                @context  = context
		@realtime = false
		@priority = :lowest
            end

            def component; context.component end

	    # Marks this task as being aperiodic (the default). To make it
	    # periodic, call #period with the required period
	    def aperiodic; @period = nil end

            # Call to make the deployer start this task when the component is
            # launched
            def start; @start = true end
            # True if this task should be started when the component is
            # started. Note that the deployer must honor the initial_state of
            # the underlying task context (i.e. call configure() if
            # initial_state is PreOperational)
            def start?; !!@start end

	    # call-seq:
	    #	period(period_in_seconds) => period_in_seconds
	    #
	    # Sets or gets the task period. Call #aperiodic to make it
	    # aperiodic
	    dsl_attribute(:period) { |value| Float(value) }

	    # Returns the Orocos scheduler constant name for this task's
	    # scheduler class. Call #realtime and #non_realtime to change the
	    # task scheduling class
	    def rtt_scheduler
		if @realtime then 'ORO_SCHED_RT'
		else 'ORO_SCHED_OTHER'
		end
	    end
	    # Marks this task as being part of the realtime scheduling class
	    def realtime; @realtime = true end
	    # Marks this task as being part of the non-realtime scheduling
	    # class
	    def non_realtime; @realtime = false end

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
        end

        class StaticDeployment
            attr_reader :component
            attr_reader :task_activities

            def initialize(component, &block)
                @task_activities = Array.new
                @component = component

                instance_eval(&block) if block_given?
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

            def corba_enabled?; @corba_enabled || (@corba_enabled.nil? && component.corba_enabled?) end
            def enable_corba;  @corba_enabled = true end
            def disable_corba; @corba_enabled = false end

            def task(name, context = name)
                name    = name.to_s
                context = context.to_s

                task_context = component.tasks.find { |t| t.name == context }
                if !task_context
                    raise ArgumentError, "no such task context #{context} defined"
                end

                deployment = TaskDeployment.new(name, task_context)
                task_activities << deployment
                deployment
            end

            def generate
                deployer = self

		# Generate the main.cpp file, which includes the ORO_main entry
		# point
		main = Generation.render_template 'main.cpp', binding
		Generation.save_automatic 'main.cpp', main
            end

            def cmake_code
                deployer = self

                Generation.render_template 'config/static_deployer.cmake', binding
            end

            dsl_attribute :main_task do |name|
                @main_task = task_by_name(name)
            end

            def task_by_name(name)
                task = task_activities.find { |t| t.name == name.to_s }
                if !task
                    raise ArgumentError, "no task #{name} defined"
                end
                task
            end

            # Sets up the TaskBrowser component and uses it to browse
            # +task_name+
            dsl_attribute :browse do |task_name|
                if browse
                    raise ArgumentError, "can browse only one task"
                elsif corba_enabled?
                    raise ArgumentError, "cannot browse and use CORBA at the same time"
                end
                @browse = task_by_name(task_name)
            end
        end
    end
end


