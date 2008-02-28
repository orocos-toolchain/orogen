module Orocos
    class Generation
	class Property
	    attr_reader :name, :type
	    def initialize(name, type)
	    end
	end

	class TaskContext
	    attr_reader :component
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
	    def aperiodic
		@period = nil
	    end

	    # Returns the Orocos scheduler constant name for this task's
	    # scheduler class. Call #realtime and #non_realtime to change
	    # the task scheduling class
	    def rtt_scheduler
		if @realtime then 'ORO_SCHED_RT'
		else 'ORO_SCHED_OTHER'
		end
	    end
	    def realtime; @realtime = true end
	    def non_realtime; @realtime = false end

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

	    dsl_attribute :priority do |value|
		if value != :highest && value != :lowest
		    Integer(value)
		else
		    value
		end
	    end

	    def event(name)
	    end

	    def property(name, type)
		if type.respond_to?(:to_str)
		    type = component.registry.get(type.to_str)
		end

		Property.new(name, type)
	    end

	    def generate
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

