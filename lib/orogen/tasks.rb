require 'utilrb/module/attr_predicate'
require 'utilrb/value_set'

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

            def used_types; [type] end

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

	    # call-seq:
	    #	doc new_doc => self
            #	doc =>  current_doc
	    #
	    # Gets/sets a string describing this object
	    dsl_attribute(:doc) { |value| value.to_s }
	end

        # Generic representation of ports. The actual ports are either
        # instance of DataPort or of BufferPort.
	class Port
	    # The port task
	    attr_reader :task
	    # The port name
	    attr_reader :name
	    # The port type
	    attr_reader :type
	    # The IO mode ('r', 'w' or 'rw')
	    attr_reader :mode

            def used_types; [type] end

            # True if it is possible to read this port
	    def read_access?; mode =~ /r/ end
            # True if it is possible to write this port
	    def write_access?; mode =~ /w/ end
            # True if this port is read-write
	    def read_write?; read_access? && write_access? end

	    def initialize(task, name, type, mode)
                name = name.to_s
		@task, @name, @type, @mode = task, name, type, mode
	    end

            # Returns the name of the Orocos class for this port (i.e.  one of
            # ReadDataPort, WriteDataPort, DataPort, ReadBufferPort, ...)
	    def orocos_class
		basename = self.class.name.gsub(/.*::/, '')
		if read_write? then basename
		elsif read_access? then "Read#{basename}"
		else "Write#{basename}"
		end
	    end

	    # call-seq:
	    #	doc new_doc => self
            #	doc =>  current_doc
	    #
	    # Gets/sets a string describing this object
	    dsl_attribute(:doc) { |value| value.to_s }
	end

        # Representation of a data port. Instances of this class are usually
        # created by TaskContext#data_port
	class DataPort < Port; end

        # Representation of a buffer port. Instances of this class are usually
        # created by TaskContext#buffer_port
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
	    # The TaskContext instance this method is part of
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
            #	doc =>  current_doc
	    #
	    # Gets/sets a string describing this object
	    dsl_attribute(:doc) { |value| value.to_s }

	    # The set of arguments of this method, as an array of [name, type,
	    # doc] elements. The +type+ objects are Typelib::Type instances.
            # 
            # See #argument
	    attr_reader :arguments

            # Defines the next argument of this method. +name+ is the argument
            # name and +type+ is either the type name as a string, or a
            # Typelib::Type object. In both cases, the required type must
            # be defined in the component, either because it is part of its
            # own toolkit or because it has been imported by a Component#load_toolkit
            # call.
	    def argument(name, type, doc = "")
		type = task.component.find_type(type)
		arguments << [name, type, doc]
		self
	    end

            def used_types
                arguments.map { |_, t, _| t }
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

        # Representation of a RTT method. Instances of this object are usually
        # created through TaskContext#method. The generated code will expect
        # the class implementation (user-visible part) to define one method, to
        # serve the call, with almost the same name that the method itself.
        #
        # For instance, the following definition
        #   method('MyMethod')
        #
        # will require the user-visible part to define
        #   [return value] myMethod([arguments]);
        #
        # (note that the first character of the method name has been set to
        # lowercase to generate the C++ method name)
        #
        # The argument list of the C++ method (the first one) can be defined
        # using Callable#argument. Its return type by using #returns. The
        # default method signature is no return type (i.e. void) and no
        # arguments.
        #
        # The name of the work and completion methods can be changed with
        # #method_name.
        #
        # For instance,
        #   method('MyMethod').
        #     argument('x', 'double', 'the target X value').
        #     argument('y', 'double', 'the target Y value').
        #     method_name('move').
        #     returns('double')
        #
        # will require the user-visible part to define
        #   double move(double x, double y);
	class Method < Callable
	    def initialize(task, name)
		super
		@method_name = self.name.dup
		method_name[0, 1] = method_name[0, 1].downcase
	    end

            def used_types
                [return_type] + super
            end

	    # The return type of this method, as a Typelib::Type object.
            # See #returns
	    attr_reader :return_type

            # Sets the return type for this method. +type+ can either be the
            # type name or a Typelib::Type object. In both cases, the required
            # type must be defined in the component, either because it is part
            # of its own toolkit or because it has been imported by a
            # Component#load_toolkit call.
	    def returns(type);
		type = task.component.find_type(type)
		@return_type = type
		self
	    end

            # Returns the C++ signature for this method. Used in code
            # generation only.
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
            #	method_name => current_name
	    #
            # Gets or sets the name of the C++ method which is to be called to
            # serve this orocos method. It default to the method name with the
            # first character set to lowercase (MyMethod becomes myMethod).
	    dsl_attribute(:method_name) { |value| value.to_s }
	end

        # Representation of a RTT command. Instances of this object are usually
        # created through TaskContext#command. The generated code will expect
        # the class implementation (user-visible part) to define two methods:
        # * a _work_ method which is called once when the command is called. This
        #   method has a boolean return type and should return true if the command
        #   started, and false otherwise.
        # * a _completion_ method which is called while the command is running.
        #   This method should return true when the command is finished and false
        #   otherwise.
        #
        # For instance, the following definition
        #   command('MyCommand')
        #
        # will require the user-visible part to define
        #   bool myCommand([arguments]);
        #   bool isMyCommandCompleted([arguments]);
        #
        # (note that the first character of the command name has been set to
        # lowercase for the work-method name)
        #
        # The argument list of the work method (the first one) can be defined
        # using Callable#argument. For the completion method, three choices are
        # available:
        # * no arguments at all
        # * the same first argument that the work method
        # * all the same arguments than the work method
        #
        # The default is to have all arguments. This can be changed using
        # #completion_no_arguments, #completion_first_argument and
        # #completion_all_arguments.
        #
        # The name of the work and completion methods can be changed with
        # #work_method_name and #completion_method_name.
        #
        # For instance,
        #   command('my_command').
        #     argument('x', 'double', 'the target X value').
        #     argument('y', 'double', 'the target Y value').
        #     work_method_name('move').
        #     completion_method_name('hasReachedTarget').
        #     completion_no_arguments.
        #
        # will require the user-visible part to define
        #   bool move(double x, double y);
        #   bool hasReachedTarget();
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
	    # first argument than the command method
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

        # Representation of TaskContext classes. This is usually created using
        # Component#task_context.
        #
        # In the generated code, two classes are actually generated:
        # * the auto-generated code is in <tt>.orogen/tasks/[name]Base.cpp</tt>
        #   and <tt>.orogen/tasks/[name]Base.hpp</tt>. These files define the
        #   various attributes related to the task context (i.e. port and
        #   attribute objects) in an <tt>[component.name]::[name]Base</tt> class.
        # * the user-visible code is in <tt>tasks/[name].cpp</tt> and
        #   <tt>tasks/[name].hpp</tt>. These files are the ones that define the
        #   <tt>[component.name]::[name]</tt> class, which is a direct subclass
        #   of the <tt>[component.name]::[name]Base</tt> class.
        #
        # By default, the +Base+ class derives from the
        # <tt>RTT::TaskContext</tt> class.  This can be changed by using the
        # #subclasses method.
        #
        # For all task context objects (ports, properties, ...) there is one
        # attribute, of the right RTT class, added to the generated TaskContext
        # subclass. The attribute name is always the _[object name], so for
        # instance the presence of the following statement
        #   data_port('time', 'double', 'w')
        #
        # will cause a <tt>WriteDataPort<double></tt> attribute named
        # <tt>_time</tt> to be added to the generated class (more specifically,
        # to the +Base+ subclass).
	class TaskContext
	    # The component this task is part of
	    attr_reader :component
	    # The task name
	    attr_reader :name
            # The subclass of TaskContext which should be used to define this
            # class
            attr_reader :superclass
            # A set of classes the TaskContext has to implement as well
            attr_reader :implemented_classes
            # Declares that this task context is a subclass of the following
            # TaskContext class. +task_context+ can either be a class name or a
            # TaskContext instance. In both cases, it must be defined in the
            # scope of the enclosing Component object -- i.e. either defined in
            # it, or imported by a Component#using_task_library call.
            def subclasses(task_context)
                @superclass = component.find_task_context task_context
                if !superclass
                    raise ArgumentError, "no such task context #{task_context}"
                end
            end

            # Declares that this task context is also a subclass of the
            # following class. +name+ does not have to be a task context class.
            def implements(name, include_file = nil)
                @implemented_classes << [name, include_file]
            end

            # True if the task context implements a parent class which matches
            # +name+. +name+ can either be a string or a regular expression.
            def implements?(name)
                @implemented_classes.any? { |class_name, _| name === class_name }
            end

            # The kind of activity that should be used by default. This is the
            # name of the corresponding method on the deployment objects
            # (:periodic, :aperiodic, :slave, :irq_driven, :fd_driven)
            #
            # This is a default value, i.e. the use of such an activity
            # is not mandatory. If #required_activity is set to true, then
            # this activity is the only kind of activity that can be used
            # with this task context.
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
            attr_predicate :required_activity?, true
            
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
                @implemented_classes = []
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
            # started (i.e. its initial state will be PreOperational instead of
            # Stopped).
            #
            # If #fixed_initial_state? returns true, then this method raises
            # ArgumentError. This is done so that it is possible to declare
            # that some task contexts's implementation require the initial
            # state to be either PreOperational or Stopped.
            def needs_configuration
                if fixed_initial_state? || superclass.fixed_initial_state?
                    raise ArgumentError, "cannot change the start state of this task context"
                end
                @initial_state = 'PreOperational'
            end

	    # The set of properties for this task
	    attr_reader :properties

            # Create a new property with the given name, type and default value
            # for this task. This returns the Property instance representing
            # the new property, whose methods can be used to configure the
            # property further. +type+ is the type name for that attribute.  It
            # can be either in Typelib notation (/std/string) or in C++
            # notation (std::string). This type must be defined either by the
            # component's own toolkit, or by toolkits imported with
            # Component#load_toolkit.
            #
            # The generated task context will have a <tt>_[property name]</tt>
            # attribute of class RTT::Property<type>.
            #
            # For instance, the following definition
            #   property('device_name', '/std/string/, '').
            #       doc 'the device name to connect to'
            #
            # Will generate a task context with a <tt>_device_name</tt>
            # attribute of type RTT::Property<std::string>.
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

            # Create a new command with the given name. Use the returned
            # Command object to configure the method further. In Orocos, a
            # command is an asynchronous method call to a task context.
            #
            # The generated class will have a <tt>_[command name]</tt>
            # attribute of class RTT::Command<>. For instance,
            #
            #   command('my_command').
            #     doc 'description of command'
            #
            # will generate an attribute <tt>_my_command</tt> of type
            # RTT::Command. The 
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
            # Typelib::Type object or a type name defined in the component
            # (i.e.  either defined in its own toolkit or loaded with
            # Component#load_toolkit). The size is, for write ports, how many
            # items can be buffered at the same time in the port. It should not
            # be provided for read-only ports.
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
	    # Typelib::Type object or a type name defined in the component (i.e.
            # either defined in its own toolkit or loaded with Component#load_toolkit).
	    #
	    # See also #buffer_port
	    def data_port(name, type, mode = 'rw'); port(DataPort, name, type, mode) end

            # Declares that this task context is designed to be woken up when
            # new data is available on one of its ports. This requires the
            # RTT::DataDrivenTask class to be installed in your installation of
            # Orocos::RTT.
            def data_driven
                subclasses "RTT::DataDrivenTask"

                @required_activity = true
                default_activity 'event_driven'
            end

            # Declares that this task context is designed to be woren up when
            # new data is available on a I/O file descriptor. This also requires
            # using the fd_driven deployment type.
            def fd_driven
                implements "RTT::FileDescriptorActivity::Provider", "rtt/FileDescriptorActivity.hpp"
                default_activity "fd_driven"
            end

            # Looks at the various data objects defined on this task, and
            # returns the list of toolkits that define them
            def used_toolkits
                types = (properties + methods + commands + ports).
                    map { |obj| obj.used_types }.
                    flatten.to_value_set.to_a

                types.map do |t|
                    tk, reg = component.used_toolkits.find do |_, reg|
                        reg.get(t.name)
                    end
                    tk
                end.compact.to_value_set.to_a
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
                used_toolkits

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

