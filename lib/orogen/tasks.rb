require 'utilrb/module/attr_predicate'

module Orocos
    module Generation
	class Property
            # The task on which this property is attached
            attr_reader :task
	    # The property name
	    attr_reader :name

	    # The property type, as a Typelib::Type object from the underlying
	    # component's type registry
	    attr_reader :type

	    # The property's default value
	    attr_reader :default_value

	    # Create a new property with the given name, type and default value
	    def initialize(task, name, type, default_value)
                name = name.to_s
		if name !~ /^\w+$/
		    raise ArgumentError, "invalid task name #{name}"
		end

		@name, @type, @default_value = name, type, default_value
	    end

	    dsl_attribute(:doc) { |value| value.to_s }
	end

	class Port
	    # The port task
	    attr_reader :task
	    # The port name
	    attr_reader :name
	    # The port type
	    attr_reader :type
	    # The IO mode ('r', 'w' or 'rw')
	    attr_reader :mode

	    def read_access?; mode =~ /r/ end
	    def write_access?; mode =~ /w/ end
	    def read_write?; read_access? && write_access? end

	    def initialize(task, name, type, mode)
                name = name.to_s
		@task, @name, @type, @mode = task, name, type, mode
	    end

	    # Returns the name of the Orocos template class for this port
	    def orocos_class
		basename = self.class.name.gsub(/.*::/, '')
		if read_write? then basename
		elsif read_access? then "Read#{basename}"
		else "Write#{basename}"
		end
	    end

	    dsl_attribute(:doc) { |value| value.to_s }
	end
	class DataPort < Port; end
	class BufferPort < Port
	    # The port size, which is the maximal number of items it can handle
	    # at the same time
	    attr_reader :size

	    def initialize(task, name, type, mode, size)
		super(task, name, type, mode)
		@size = size
	    end
	end

	class Callable
	    # The task this method is part of
	    attr_reader :task
	    # The method name
	    attr_reader :name

	    def initialize(task, name)
                name = name.to_s
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
		type = task.component.find_type(type)
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
		@method_name = self.name.dup
		method_name[0, 1] = method_name[0, 1].downcase
	    end

	    # The return type of this method
	    attr_reader :return_type

	    # Set the return type for this method
	    def returns(type);
		type = task.component.find_type(type)
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

	    # Create a new callable object on the specified task and with the
	    # specified name. The setup of the callable should be done by
	    # calling the various setup methods after the object has been
	    # created.
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
	    
	    # The completion method for this command has no argument at all
	    def completion_no_arguments;   @completion_signature_type = :no_arguments   end
	    # The completion method for this command will be given the same
	    # first argument of the command method
	    def completion_first_argument; @completion_signature_type = :first_argument end
	    # The completion method for this command will be given the same
	    # arguments than the command method
	    def completion_all_arguments;  @completion_signature_type = :all_arguments  end

	    # A string representing the signature for the C++ work method.  If
	    # +with_names+ is true, the name of the method and the names of the
	    # arguments are included in the string.
	    def work_signature(with_names = true)
		argument_signature(with_names)
	    end
	    
	    # A string representing the signature for the C++ completion
	    # method.  If +with_names+ is true, the name of the method and the
	    # names of the arguments are included in the string.
	    #
	    # The result depends on the completion_signature_type attribute,
	    # which can be changed by the completion_no_arguments,
	    # completion_first_argument and completion_all_arguments methods.
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
            # The subclass of TaskContext which should be used to define this
            # class
            attr_reader :superclass
            # Declares that this task context is a subclass of the following type
            def subclasses(task_context)
                @superclass = component.find_task_context task_context
                if !superclass
                    raise ArgumentError, "no such task context #{task_context}"
                end
            end

            # The kind of activity that should be used by default. This is the
            # name of the corresponding method on the deployment objects
            # (:periodic, :aperiodic, :slave, :irq_driven, :fd_driven)
            dsl_attribute :default_activity do |type, *args|
                if required_activity? && @default_activity
                    raise ArgumentError, "the #{default_activity[0]} activity is required, you cannot change it"
                end

                type = type.to_sym
                if !TaskDeployment.method_defined?(type)
                    raise ArgumentError, "#{type} is not a valid activity type"
                end
                [type, *args]
            end
            # True if the current value of default_activity is actually
            # required by the task context implementation
            attr_predicate :required_activity?
            
            # True if this task context is defined by one of our dependencies.
            attr_predicate :external_definition?, true

            # The name of the header file containing the C++ code which defines
            # this task context
            def header_file
                if external_definition?
                    library_name, name = self.name.split("::")
                    "#{library_name.downcase}/#{name}.hpp"
                else
                    "tasks/#{basename}.hpp"
                end
            end

            # Returns the name without an eventual library name
            def basename
                library_name, name = self.name.split("::")
                name || library_name
            end

            # True if we are generating for Linux
            def linux?;     component.linux? end
            # True if we are generating for Xenomai
            def xenomai?;   component.xenomai? end

            def class_name
                name
            end

	    # Create a new task context in the given component and with
	    # the given name. If a block is given, it is evaluated
	    # in the context of the newly created TaskContext object.
	    #
	    # TaskContext objects should not be created directly. You should
	    # use Component#task_context for that.
	    def initialize(component, name)
		if name !~ /^(\w+::)?\w+$/
		    raise ArgumentError, "invalid task name #{name}"
		end

		@component  = component
                @superclass = component.default_task_superclass
		@name = name

		@properties = Array.new
		@methods    = Array.new
		@commands   = Array.new
		@ports	    = Array.new
                @initial_state = 'Stopped'
	    end

	    def pretty_print(pp)
		pp.text to_s
	    end

	    # Raises ArgumentError if an object named +name+ is already present
	    # in the set attribute +set_name+. 
	    #
	    # This is an internal helper method
	    def check_uniqueness(set_name, name) # :nodoc:
		set = send(set_name)
		if set.find { |o| o.name == name }
		    raise ArgumentError, "there is already a #{name} in #{set_name}"
		end
	    end
	    private :check_uniqueness

            # Initial state of the task. May be either 'Stopped' or
            # 'PreOperational'.  By default, it is 'Stopped'. Call
            # #needs_configuration to make it PreOperational.  Some task
            # contexts do not allow to change the start state, in which case
            # their #fixed_initial_state? method returns true.
            attr_reader :initial_state

            # If true, it is not possible to change the start state of this
            # component
            def fixed_initial_state?; !!@fixed_initial_state || (superclass && superclass.fixed_initial_state?) end
            
            # Declares that it is not possible to change the start state of
            # this component
            def fixed_initial_state; @fixed_initial_state = true end


            # Declares that this task needs to be configured before it is
            # started (i.e. its initial state will be PreOperational instead
            # of Stopped)
            def needs_configuration
                if fixed_initial_state? || superclass.fixed_initial_state?
                    raise ArgumentError, "cannot change the start state of this task context"
                end
                @initial_state = 'PreOperational'
            end

	    # The set of properties for this task
	    attr_reader :properties

	    # Create a new property with the given name, type and default value
	    # for this task. This returns the Property representing the new
	    # property
	    def property(name, type, default_value = nil)
		check_uniqueness(:properties, name)
		type = component.find_type(type)

		properties << Property.new(self, name, type, default_value)
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
		check_uniqueness :methods, name
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
		check_uniqueness :commands, name
		commands << Command.new(self, name)
		commands.last
	    end

	    # The set of IO ports for this task context. These are either
	    # DataPort and BufferPort objects
	    attr_reader :ports

	    # Inserts a new port of the given class, name, type and with the
	    # given mode.
	    #
	    # This is an internal helper method used to implement #data_port
	    # and #buffer_port
	    def port(klass, name, type, mode, *additional_args) # :nodoc:
		check_uniqueness(:ports, name)
		mode = mode.to_s
		unless %w{r w rw }.any? { |v| v == mode }
		    raise ArgumentError, "invalid mode specification #{mode}. Expected one of: r, w, rw"
		end
		type = component.find_type(type)

		ports << klass.new(self, name, type, mode, *additional_args)
		ports.last
	    end
	    private :port

	    # call-seq:
	    #	buffer_port 'name', '/type', 'r'
	    #	buffer_port 'name', '/type', buffer_size, 'rw'
	    #	buffer_port 'name', '/type', buffer_size, 'w'
	    #
	    # Add a new buffer port with the given name, type, size and mode.
	    # Mode defines in what direction the port accept data: it is a
	    # string and can be one of r, w, and rw. +type+ is either a
	    # Typelib::Type object or a type name defined in the component type
	    # registry. The size is, for write ports, how many items can be
	    # buffered at the same time in the port. It should not be provided
	    # for read-only ports.
	    #
	    # See also #data_port
	    def buffer_port(name, type, *spec)
		spec << 'r' if spec.empty?

		mode_or_size = spec.shift
		if mode_or_size.respond_to?(:to_str)
		    if mode_or_size.to_str != 'r'
			raise ArgumentError, "write access needs a buffer size to be defined"
		    end
		    mode = 'r'
		    size = nil
		else
		    mode = spec.shift.to_s
		    if mode == 'r'
			raise ArgumentError, "buffer size provided for read-only buffer port"
                    elsif !mode
                        raise ArgumentError, "mode not specified"
		    end

		    size = Integer(mode_or_size)
		end

		port(BufferPort, name, type, mode, size) 
	    end

	    # Add a new buffer port with the given name, type and mode.
	    # Mode defines in what direction the port accept data: it is a
	    # string and can be one of r, w, and rw. +type+ is either a
	    # Typelib::Type object or a type name defined in the component type
	    # registry.
	    #
	    # See also #buffer_port
	    def data_port(name, type, mode = 'rw'); port(DataPort, name, type, mode) end

            # Declares that this task context is designed to be woken up when
            # new data is available on one of its ports.
            def data_driven
                subclasses "RTT::DataDrivenTask"

                @required_activity = true
                default_activity 'event_driven'
            end

	    # Generate the code files for this task. This builds to classes:
	    #
	    # * a #{task.name}Base class in .orogen/tasks/#{task.name}Base.{cpp,hpp}
	    #   which is the automatically generated part of the task.
	    # * a #{task.name} class in tasks/#{task.name}.{cpp,hpp} which is
	    #   the user-provided part of the task. This class is a public
	    #   subclass of the Base class.
	    def generate
                return if external_definition?

		# Make this task be available in templates as 'task'
		task = self
	    
		base_code_cpp = Generation.render_template 'tasks', 'TaskBase.cpp', binding
		base_code_hpp = Generation.render_template 'tasks', 'TaskBase.hpp', binding
		Generation.save_automatic "tasks", "#{basename}Base.cpp", base_code_cpp
		Generation.save_automatic "tasks", "#{basename}Base.hpp", base_code_hpp

		code_cpp = Generation.render_template "tasks", "Task.cpp", binding
		code_hpp = Generation.render_template "tasks", "Task.hpp", binding
		Generation.save_user "tasks", "#{basename}.cpp", code_cpp
		Generation.save_user "tasks", "#{basename}.hpp", code_hpp

		self
	    end
	end
    end
end

