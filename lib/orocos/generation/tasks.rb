module Orocos
    module Generation
	class Property
	    # The property name
	    attr_reader :name

	    # The property type, as a Typelib::Type object from the underlying
	    # component's type registry
	    attr_reader :type

	    # The property's default value
	    attr_reader :default_value

	    # Create a new property with the given name, type and default value
	    def initialize(name, type, default_value)
		if name !~ /^\w+$/
		    raise ArgumentError, "invalid task name #{name}"
		end

		@name, @type, @default_value = name, type, default_value
	    end

	    dsl_attribute(:doc) { |value| value.to_s }
	end

	class Callable
	    # The task this method is part of
	    attr_reader :task
	    # The method name
	    attr_reader :name

	    def initialize(task, name)
		if name !~ /^\w+$/
		    raise ArgumentError, "invalid task name #{name}"
		end

		@task = task
		@name = name
		@arguments = []
	    end

	    # call-seq:
	    #	doc new_doc => self
	    #
	    # Set the documentation string and returns self
	
	    dsl_attribute(:doc) { |value| value.to_s }

	    # The set of arguments of this method, as an array of [name, type,
	    # doc] elements
	    attr_reader :arguments

	    # Defines the next argument of this method
	    def argument(name, type, doc = "")
		if type.respond_to?(:to_str)
		    type = task.component.registry.build(type.to_str)
		end
		arguments << [name, type, doc]
		self
	    end

	    # Returns the argument part of the C++ signature for this callable
	    def argument_signature(with_names = true)
		arglist = arguments.map do |name, type, doc|
		    type = type.full_name('::', true)
		    if with_names then "#{type} #{name}"
		    else type
		    end
		end

		"(" << arglist.join(", ") << ")"
	    end
	end

	class Method < Callable
	    # The C++ method name to be called to serve this Orocos method.
	    # This defaults to +name+, but you can customize it by using
	    # #method_name
	    attr_reader :method_name

	    def initialize(task, name)
		super
		@method_name = name.dup
		method_name[0, 1] = method_name[0, 1].downcase
	    end

	    # The return type of this method
	    attr_reader :return_type

	    # Set the return type for this method
	    def returns(type);
		if type.respond_to?(:to_str)
		    type = task.component.registry.build(type.to_str)
		end

		@return_type = type
		self
	    end

	    def signature(with_names = true)
		result = ""
		if return_type
		    result << return_type.full_name('::', true)
		else
		    result << "void"
		end
		result << argument_signature(with_names)
	    end

	    # call-seq:
	    #	method_name new_name => self
	    #
	    # Sets the name of the C++ method which is to be called to serve
	    # this orocos method. It default to the method name itself.

	    dsl_attribute(:method_name) { |value| value.to_s }
	end

	class Command < Callable
	    # The C++ method name to be called to serve this Orocos command.
	    # This defaults to +name+, but you can customize it by using
	    # #method_name
	    dsl_attribute(:work_method_name) { |name| name.to_s }
	    # The C++ method name to be called to serve this Orocos command.
	    # This defaults to is+name+Completed, but you can customize it by
	    # using #method_name
	    dsl_attribute(:completion_method_name) { |name| name.to_s }

	    def initialize(task, name)
		super
		@work_method_name = name.dup
		work_method_name[0, 1] = work_method_name[0, 1].downcase
		@completion_method_name = "is#{name}Completed"

		@completion_signature_type = :all_arguments
	    end

	    # Which kind of signature do we want for the completion method ?
	    # It is either +:no_arguments+, +:first_argument+ or +:all_arguments+
	    # Use #completion_no_arguments, #completion_first_argument or #completion_all_arguments
	    # to change it
	    attr_reader :completion_signature_type

	    def completion_no_arguments;   @completion_signature_type = :no_arguments   end
	    def completion_first_argument; @completion_signature_type = :first_argument end
	    def completion_all_arguments;  @completion_signature_type = :all_arguments  end

	    def work_signature(with_names = true)
		argument_signature(with_names)
	    end
	    def completion_signature(with_names = true)
		case completion_signature_type
		when :no_arguments then "()"
		when :first_argument
		    argument_signature(with_names).gsub(/,.*\)$/, ")")
		when :all_arguments; argument_signature(with_names)
		end
	    end
	end

	class TaskContext
	    # The component this task is part of
	    attr_reader :component
	    # The task name
	    attr_reader :name

	    def initialize(component, name, &block)
		if name !~ /^\w+$/
		    raise ArgumentError, "invalid task name #{name}"
		end

		@component = component
		@name = name
		@realtime = true
		@priority = :lowest

		@properties = Array.new
		@methods    = Array.new
		@commands   = Array.new

		instance_eval(&block) if block
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

	    def pretty_print(pp)
		pp.text to_s
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

	    # The set of properties for this task
	    attr_reader :properties

	    # Create a new property with the given name, type and default value
	    # for this task. This returns the Property representing the new
	    # property
	    def property(name, type, default_value = '')
		if properties.find { |p| p.name == name }
		    raise ArgumentError, "there is already a #{name} property"
		end

		if type.respond_to?(:to_str)
		    type = component.registry.build(type.to_str)
		end

		properties << Property.new(name, type, default_value)
		properties.last
	    end

	    # The set of methods for this task.
	    attr_reader :methods

	    # Create a new method with the given name. Use the returned Method
	    # object to configure the method further.
	    #
	    # In Orocos, a method is a synchronous method call to a task context:
	    # the caller will block until the method's procedure is called
	    def method(name)
		if methods.find { |m| m.name == name }
		    raise ArgumentError, "there is already a #{name} method"
		end

		methods << Method.new(self, name)
		methods.last
	    end


	    # The set of commands for this task.
	    attr_reader :commands

	    # Create a new command with the given name. Use the returned Command
	    # object to configure the method further.
	    #
	    # In Orocos, a command is an asynchronous method call to a task
	    # context.
	    def command(name)
		if commands.find { |c| c.name == name }
		    raise ArgumentError, "there is already a #{name} command"
		end

		commands << Command.new(self, name)
		commands.last
	    end

	    # Generate the code files for this task. This builds to classes:
	    #
	    # * a #{task.name}Base class in .orogen/tasks/#{task.name}Base.{cpp,hpp}
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
    end
end

