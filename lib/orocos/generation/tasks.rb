module Orocos
    class Generation
	class Property
	    attr_reader :name, :type
	    def initialize(name, type)
	    end
	end

	class TaskContext
	    # The component this task is part of
	    attr_reader :component
	    # The task name
	    attr_reader :name

	    def initialize(component, name, &block)
		@component = component
		@name = name
		@realtime = true
		@priority = :lowest
		instance_eval(&block) if block
	    end

	    dsl_attribute :period do |value|
		Float(value)
	    end

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

	    # Make this task as being of the highest priority allowed by the
	    # underlying OS
	    def highest_priority; @priority = :highest end
	    # Make this task as being of the lowest priority allowed by the
	    # underlying OS
	    def lowest_priority;  @priority = :lowest end
	    # call-seq:
	    #	priority prio => new_prio
	    #
	    # Sets the task priority as an integer value. Allowed values are
	    # OS-specific, and for now the allowed range is unfortunately not
	    # available from Ruby
	    dsl_attribute(:priority) { |value| Integer(value) }

	    def event(name)
	    end

	    def property(name, type)
		if type.respond_to?(:to_str)
		    type = component.registry.get(type.to_str)
		end

		Property.new(name, type)
	    end

	    # Generate the code files for this task. This builds to classes:
	    #
	    # * a #{task.name}Base class in .orocos/tasks/#{task.name}Base.{cpp,hpp}
	    #   which is the automatically generated part of the task.
	    # * a #{task.name} class in tasks/#{task.name}.{cpp,hpp} which is
	    #   the user-provided part of the task. This class is a public
	    #   subclass of the Base class.
	    def generate
		# Make this task be available in templates as 'task'
		task = self
	    
		base_code_cpp = Generation.render_template 'tasks', 'TaskBase.cpp', binding
		base_code_hpp = Generation.render_template 'tasks', 'TaskBase.hpp', binding
		Generation.save_automatic "tasks", "#{name}Base.cpp", base_code_cpp
		Generation.save_automatic "tasks", "#{name}Base.hpp", base_code_hpp

		code_cpp = Generation.render_template "tasks", "Task.cpp", binding
		code_hpp = Generation.render_template "tasks", "Task.hpp", binding
		Generation.save_user "tasks", "#{name}.cpp", code_cpp
		Generation.save_user "tasks", "#{name}.hpp", code_hpp

		self
	    end
	end

	class Event
	end

	def task_context(name, &block)
	    if tasks.find { |t| t.name == name }
		raise ArgumentError, "there is already a #{name} task"
	    end

	    new_task = TaskContext.new(self, name, &block)
	    new_task.generate
	    tasks << new_task
	end
    end
end

