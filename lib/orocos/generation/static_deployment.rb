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
		# Generate the main.cpp file, which includes the ORO_main entry
		# point
		main = Generation.render_template 'main.cpp', binding
		Generation.save_automatic 'main.cpp', main
            end
            def cmake_code
                Generation.render_template 'config/static_deployer.cmake', binding
            end
        end
    end
end


