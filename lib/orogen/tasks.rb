require 'utilrb/module/attr_predicate'
require 'utilrb/value_set'

module Orocos
    def self.validate_toplevel_type(type)
        if type < Typelib::ArrayType
            raise ArgumentError, "array types can be used only in a structure"
        elsif type < Typelib::NumericType && !Typelib::Registry.base_rtt_type?(type)
            raise ArgumentError, "#{type.name} cannot be used as a toplevel type"
        end
    end

    module Generation
        ACTIVITY_TYPES = {
            :fd_driven    => 'FileDescriptorActivity',
            :irq_driven   => 'IRQActivity',
            :slave        => 'SlaveActivity',
            :periodic     => 'PeriodicActivity',
            :triggered    => 'NonPeriodicActivity',
            :sequential   => 'SequentialActivity'
        }

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
		    raise ArgumentError, "invalid property name #{name}"
		end

                type = task.component.find_type(type)
                Orocos.validate_toplevel_type(type)

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
        # instance of InputPort or OutputPort
	class Port
	    # The port task
	    attr_reader :task
	    # The port name
	    attr_reader :name
	    # The port type
	    attr_reader :type

            def used_types; [type] end

	    def initialize(task, name, type)
                if !name.kind_of?(Regexp)
                    name = name.to_s
                    if name !~ /^\w+$/
                        raise ArgumentError, "invalid port name #{name}"
                    end
                end

                type = task.component.find_type(type)
                Orocos.validate_toplevel_type(type)
                if type.name == "/std/vector<double>"
                    Orocos::Generation.warn "#{type.name} is used as the port type for #{name}, logging it will not be possible"
                end
		@task, @name, @type = task, name, type
	    end

	    # call-seq:
	    #	doc new_doc => self
            #	doc =>  current_doc
	    #
	    # Gets/sets a string describing this object
	    dsl_attribute(:doc) { |value| value.to_s }
	end

        class OutputPort < Port
            # Returns the name of the Orocos class for this port (i.e.  one of
            # ReadDataPort, WriteDataPort, DataPort, ReadBufferPort, ...)
	    def orocos_class; "RTT::OutputPort" end
        end

        class InputPort < Port
            # Returns the name of the Orocos class for this port (i.e.  one of
            # ReadDataPort, WriteDataPort, DataPort, ReadBufferPort, ...)
	    def orocos_class; "RTT::InputPort" end
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
            # Typelib::Type object. In both cases, the required type must be
            # defined in the component, either because it is part of its own
            # toolkit or because it has been imported by a
            # Component#load_toolkit call.
            #
            # Note that Orocos::RTT does not support having more than 4
            # arguments for a method, and trying that will therefore raise an
            # error
	    def argument(name, type, doc = "")
                if arguments.size == 4
                    raise ArgumentError, "Orocos does not support having more than 4 arguments for a method"
                end

		type = task.component.find_type(type)
                Orocos.validate_toplevel_type(type)
		arguments << [name, type, doc]
		self
	    end


            def used_types
                arguments.map { |_, t, _| t }
            end

	    # Returns the argument part of the C++ signature for this callable
	    def argument_signature(with_names = true)
		arglist = arguments.map do |name, type, doc|

		    arg = type.full_name('::', true)
                    if !(type < Typelib::NumericType)
                        arg += " const &"
                    end

		    if with_names then "#{arg} #{name}"
		    else arg
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
                [return_type].compact + super
            end

	    # The return type of this method, as a Typelib::Type object.
            # See #returns
	    attr_reader :return_type

            # Sets the return type for this method. +type+ can either be the
            # type name or a Typelib::Type object. In both cases, the required
            # type must be defined in the component, either because it is part
            # of its own toolkit or because it has been imported by a
            # Component#load_toolkit call.
	    def returns(type)
                if type
                    type = task.component.find_type(type)
                    Orocos.validate_toplevel_type(type)
                end
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
		@work_method_name = self.name.dup
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
        #   output_port('time', 'double')
        #
        # will cause a <tt>OutputPort<double></tt> attribute named
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
            # A set of Port objects that can be created at runtime
            attr_reader :dynamic_ports

            # Call to declare that this task model is not meant to run in
            # practice
            def abstract; @abstract = true; end
            # True if this task model is only meant to declare an interface, and
            # should not be deployed
            def abstract?; @abstract end

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
                class_name == name ||
                    (superclass && superclass.name == name) ||
                    @implemented_classes.any? { |class_name, _| name === class_name }
            end

            # The kind of activity that must be used for this task context. This
            # is the name of the corresponding method on the deployment objects.
            # See ACTIVITY_TYPES for the list of known activity types.
            #
            # See also #default_activity
            dsl_attribute :required_activity do |type, *args|
                if respond_to?(type.to_sym)
                    send(type.to_sym)
                else
                    default_activity type, *args
                end
                self.required_activity = true
            end

            # The kind of activity that should be used by default. This is the
            # name of the corresponding method on the deployment objects
            # (:periodic, :aperiodic, :slave, :irq_driven, :fd_driven)
            #
            # This is a default value, i.e. the use of such an activity
            # is not mandatory. If #required_activity is set to true, then
            # this activity is the only kind of activity that can be used
            # with this task context.
            #
            # See also #required_activity
            dsl_attribute :default_activity do |type, *args|
                if required_activity? && @default_activity
                    raise ArgumentError, "the #{default_activity[0]} activity is required, you cannot change it"
                end

                type = type.to_sym
                if !ACTIVITY_TYPES.has_key?(type)
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
                    "#{component.name.downcase}/#{basename}.hpp"
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
                if name == component.name
                    raise ArgumentError, "tasks and projects must not have the same name"
                elsif name !~ /^(\w+::)?\w+$/
		    raise ArgumentError, "invalid task name #{name}"
		end

		@component  = component
                @superclass = component.default_task_superclass
                @implemented_classes = []
		@name = name
                # This is an array, as we don't want to have it reordered
                # unnecessarily
                @states = Array.new
                default_activity 'triggered'

		@properties = Array.new
		@methods    = Array.new
		@commands   = Array.new
		@ports	    = Array.new
                @dynamic_ports = Array.new
                @event_ports = Array.new
                @initial_state = 'Stopped'

                @fixed_initial_state = false
                @needs_configuration = false
	    end

	    def pretty_print(pp)
		pp.text to_s
	    end

            # Returns the object in +set_name+ for which #name returns +name+,
            # or ArgumentError if there is none
            def get_object(set_name, name)
                set = send("all_#{set_name}")
		obj = set.find { |o| o.name == name }
                if !obj
		    raise ArgumentError, "there is no #{name} in #{set_name}"
		end
                obj
            end

	    # Raises ArgumentError if an object named +name+ is already present
	    # in the set attribute +set_name+. 
	    #
	    # This is an internal helper method
	    def check_uniqueness(set_name, name) # :nodoc:
		set = send("all_#{set_name}")
		if set.find { |o| o.name == name }
		    raise ArgumentError, "there is already a #{name} in #{set_name}"
		end
	    end
	    private :check_uniqueness

            # If true, then the initial state of this class cannot be specified.
            # For orogen-declared tasks, it is the same as
            # #needs_configuration?. This mechanism is here for classes that
            # have not been generated by orogen and either have a no way to
            # specify the initial state, or a non-standard one.
            def fixed_initial_state?; @fixed_initial_state || needs_configuration? || (superclass.fixed_initial_state? if superclass) end

            # Declares that the initial state of this class cannot be specified.
            # For orogen-declared tasks, it is the same as
            # #needs_configuration?. This mechanism is here for classes that
            # have not been generated by orogen and either have a no way to
            # specify the initial state, or a non-standard one.
            def fixed_initial_state; @fixed_initial_state = true end

            # If true, the task context will start in the PreOperational state,
            # and will not be able to run until configure() has been called and
            # returned true.
            #
            # When subclassing, it is NOT possible to have a subclass starting
            # in the Stopped state while its superclass starts from
            # PreOperational.
            def needs_configuration?; @needs_configuration || (superclass.needs_configuration? if superclass) end

            # Declares that this task needs to be configured before it is
            # started (i.e. its initial state will be PreOperational instead of
            # Stopped).
            #
            # If #fixed_initial_state? returns true, then this method raises
            # ArgumentError. This is done so that it is possible to declare
            # that some task contexts's implementation require the initial
            # state to be either PreOperational or Stopped.
            def needs_configuration
                if superclass && superclass.fixed_initial_state?
                    raise ArgumentError, "cannot change the start state of this task context: the superclass #{superclass.name} does not allow it"
                elsif fixed_initial_state? && !needs_configuration?
                    raise ArgumentError, "cannot change the start state of this task context: #fixed_initial_state has been specified for it"
                end
                @needs_configuration = true
            end

	    # The set of properties for this task
	    def all_properties; @properties + (superclass ? superclass.all_properties : []) end
            def self_properties; @properties end

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
		check_uniqueness :properties, name
		type = component.find_type(type)

		@properties << Property.new(self, name, type, default_value)
		@properties.last
	    end

	    # The set of methods defined on this task
	    def all_methods; @methods + (superclass ? superclass.all_methods : []) end
            # Methods that are added by this task context (i.e. methods that are
            # defined there but are not present in the superclass)
            def new_methods
                super_names = superclass.all_methods.map(&:name).to_set
                @methods.find_all do |t|
                    !super_names.include?(t)
                end
            end
            # Methods that are overloaded by this task context (i.e. methods
            # that are defined there and are also present in the superclass)
            def overloaded_methods
                super_names = superclass.all_methods.map(&:name).to_set
                @methods.find_all do |t|
                    super_names.include?(t)
                end
            end

            def self_methods
                @methods
            end

            # Asks orogen to implement the extended state support interface in
            # the Base class. This adds:
            #  * a 'state' output port in which the current task's state is written
            #  * an enumeration type named CLASS_NAME_STATES in which one value
            #    is defined for each states
            #
            # Note that, for all of this to work, it is actually required that
            # all the hooks overloaded in the task's class call their parent in
            # the call chain.
            def extended_state_support
                state_port = each_port.find { |p| p.name == "state" }
                if state_port
                    if state_port.kind_of?(InputPort)
                        raise ArgumentError, 
                            "there is already an input port called 'state', cannot enable extended state support"
                    elsif state_port.typename != "/int"
                        raise ArgumentError, 
                            "there is already an output port called 'state', but it is not of type 'int' (found #{state_port.typename}"
                    end
                else
                    output_port 'state', '/int'
                end

                # Force toolkit generation. The toolkit code will take care of
                # generating the state enumeration type for us
                component.toolkit(true)

                @extended_state_support = true
            end

            # True if the extended state support is enabled
            def extended_state_support?
                @extended_state_support || (superclass.extended_state_support? if superclass)
            end

            # Returns true if the given state name is already used
            def state?(name)
                state_kind(name) || (superclass.state?(name.to_s) if superclass)
            end

            STATE_TYPES = [ :toplevel, :runtime, :error, :fatal ]

            # Internal method for state definition
            def define_state(name, type) # :nodoc:
                name = name.to_s
                type = type.to_sym
                if !STATE_TYPES.include?(type)
                    raise ArgumentError, "unknown state type #{type.inspect}"
                end

                if !extended_state_support?
                    extended_state_support
                end

                if kind = state_kind(name.to_s)
                    if kind != type
                        raise ArgumentError, "state #{name} is already defined as #{kind}, cannot overload into #{type}"
                    end
                else
                    @states << [name, type]
                    @states = @states.sort_by { |n, _| n }
                end
            end

            # Returns what kind of state +name+ is
            def state_kind(name) # :nodoc:
                if s = each_state.find { |n, t| n == name }
                    s[1]
                end
            end

            # Returns the type name for the state enumeration
            def state_type_name # :nodoc:
                "#{basename}_STATES"
            end

            # Returns the C++ value name for the given state when defined
            # globally
            def state_global_value_name(state_name, state_type) # :nodoc:
                "#{basename}_#{state_name.upcase}"
            end

            # Returns the C++ value name for the given state when defined in the
            # associated class scope.
            def state_local_value_name(state_name, state_type) # :nodoc:
                state_name.upcase
            end

            ##
            # :method: each_runtime_state
            #
            # Enumerates all the runtime states
            #
            # See also #each_error_state, #each_fatal_state and #each_state

            ##
            # :method: each_error_state
            #
            # Enumerates all error states defined for this task context
            #
            # See also #each_runtime_state, #each_fatal_state, and #each_state

            ##
            # :method: each_fatal_state
            #
            # Enumerates all error states defined for this task context
            #
            # See also #each_runtime_state, #each_error_state and #each_state

            STATE_TYPES.each do |type|
                class_eval <<-EOD
                def each_#{type}_state
                    if block_given?
                        each_state do |name, type|
                            yield(name) if type == :#{type}
                        end
                    else
                        enum_for(:each_#{type}_state)
                    end
                end
                EOD
            end

            # Enumerates each state defined on this task context.
            def each_state(&block)
                if block_given?
                    superclass.each_state(&block) if superclass
                    @states.each(&block)
                else
                    enum_for(:each_state)
                end
            end

            # call-seq:
            #   states => set of states
            #
            # Declares a toplevel state. It should be used only to declare RTT's
            # TaskContext states.
            def states(*state_names) # :nodoc:
                if state_names.empty?
                    return @states
                end

                state_names.each do |name|
                    define_state(name, :toplevel)
                end
            end

            # Declares a certain number of runtime states
            #
            # This method will do nothing if it defines a state that is already
            # defined by one of the superclasses.
            #
            # See #error_states, #each_state, #each_runtime_state
            def runtime_states(*state_names)
                state_names.each do |name|
                    define_state(name, :runtime)
                end
            end

            # Declares a certain number of runtime error states
            #
            # This method will do nothing if it defines a state that is already
            # defined by one of the superclasses.
            #
            # See #runtime_states, #each_state, #each_error_state
            def error_states(*state_names)
                state_names.each do |name|
                    define_state(name, :error)
                end
            end

            # Declares a certain number of fatal error states
            #
            # This method will do nothing if it defines a state that is already
            # defined by one of the superclasses.
            #
            # See #runtime_states, #error_states, #each_state, #each_error_state
            def fatal_states(*state_names)
                state_names.each do |name|
                    define_state(name, :fatal)
                end
            end

            # This method is an easier way use boost::shared_ptr in a task
            # context interface. For instance, instead of writing
            #
            #   input_port 'image', '/boost/shared_ptr</Image>'
            #
            # you can write
            #
            #   input_port 'image', shared_ptr('/Image')
            #
            # Additionally, this method makes sure that the corresponding type
            # is actually defined on the project's toolkit.
            def shared_ptr(name)
                base_type = component.find_type(name)
                full_name = "/boost/shared_ptr<#{base_type.name}>"
                begin
                    component.find_type(full_name)
                rescue Typelib::NotFound
                    component.toolkit { shared_ptr(name) }
                    component.find_type(full_name)
                end
            end

            # This method is an easier way use boost::shared_ptr in a task
            # context interface. For instance, instead of writing
            #
            #   input_port 'image', '/RTT/ReadOnlyPointer</Image>'
            #
            # you can write
            #
            #   input_port 'image', ro_ptr('/Image')
            #
            # Additionally, this method makes sure that the corresponding type
            # is actually defined on the project's toolkit.
            def ro_ptr(name)
                base_type =
                    begin
                        component.find_type(name)
                    rescue Typelib::NotFound
                        raise ArgumentError, "type #{name} is not available"
                    end

                full_name = "/RTT/ReadOnlyPointer<#{base_type.name}>"
                begin
                    component.find_type(full_name)
                rescue Typelib::NotFound
                    component.toolkit { ro_ptr(name) }
                    component.find_type(full_name)
                end
            end

	    # Create a new method with the given name. Use the returned Method
	    # object to configure the method further.
	    #
	    # In Orocos, a method is a synchronous method call to a task context:
	    # the caller will block until the method's procedure is called
	    def method(name)
		@methods << Method.new(self, name)
		@methods.last
	    end

	    # The set of commands for this task.
	    def all_commands; @commands + (superclass ? superclass.all_commands : []) end
            def new_commands
                super_names = superclass.all_commands.map(&:name).to_set
                @commands.find_all do |t|
                    !super_names.include?(t)
                end
            end
            def overloaded_commands
                super_names = superclass.all_commands.map(&:name).to_set
                @commands.find_all do |t|
                    !super_names.include?(t)
                end
            end
            def self_commands
                @commands
            end

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
		@commands << Command.new(self, name)
		@commands.last
	    end

	    # The set of IO ports for this task context. These are either
	    # OutputPort and InputPort objects
            def all_ports; @ports + (superclass ? superclass.all_ports : []) end
            def self_ports; @ports end

            def each_port(only_self = false, &block)
                if block_given?
                    if !only_self && superclass
                        superclass.each_port(false, &block)
                    end
                    @ports.each(&block)
                else
                    enum_for(:each_port, only_self)
                end
            end

	    # call-seq:
	    #	output_port 'name', '/type'
	    #
            # Add a new write port with the given name and type, and returns the
            # corresponding OutputPort object.
	    #
	    # See also #input_port
	    def output_port(name, type)
		check_uniqueness(:ports, name)
                @ports << OutputPort.new(self, name, type)
                @ports.last
            rescue Typelib::NotFound
                raise ConfigError, "type #{type} is not declared"
	    end

            # Enumerates the output ports available on this task context. If no
            # block is given, returns the corresponding enumerator object.
            def each_output_port(&block)
                if block_given?
                    @ports.each do |p|
                        yield(p) if p.kind_of?(OutputPort)
                    end
                    if superclass
                        superclass.each_output_port(&block) if superclass
                    end
                else
                    enum_for(:each_output_port)
                end
            end

	    # call-seq:
	    #	input_port 'name', '/type'
	    #
            # Add a new write port with the given name and type, and returns the
            # corresponding InputPort object.
	    #
	    # See also #output_port
	    def input_port(name, type)
		check_uniqueness(:ports, name)
                @ports << InputPort.new(self, name, type)
                @ports.last
            rescue Typelib::NotFound
                raise ConfigError, "type #{type} is not declared"
            end

            # call-seq:
            #   dynamic_input_port name_regex, typename
            #
            # Declares that a port whose name matches name_regex can be declared
            # at runtime, with the type. This is not used by orogen himself, but
            # can be used by potential users of the orogen specification.
            def dynamic_input_port(name, type)
                dynamic_ports << InputPort.new(self, name, type)
                dynamic_ports.last
            end

            # call-seq:
            #   dynamic_output_port name_regex, typename
            #
            # Declares that a port whose name matches name_regex can be declared
            # at runtime, with the type. This is not used by orogen himself, but
            # can be used by potential users of the orogen specification.
            def dynamic_output_port(name, type)
                dynamic_ports << OutputPort.new(self, name, type)
                dynamic_ports.last
            end

            def dynamic_port?(name, type)
                dynamic_input_port?(name, type) || dynamic_output_port?(name, type)
            end

            def find_dynamic_input_ports(name, type)
                dynamic_ports.find_all { |p| p.kind_of?(InputPort) && p.type == component.find_type(type) && p.name === name }
            end

            # Returns true if an input port of the given name and type could be
            # created at runtime.
            def dynamic_input_port?(name, type)
                !find_dynamic_input_ports(name, type).empty?
            end

            def find_dynamic_output_ports(name, type)
                dynamic_ports.find_all { |p| p.kind_of?(OutputPort) && p.type == component.find_type(type) && p.name === name }
            end

            # Returns true if an output port of the given name and type could be
            # created at runtime.
            def dynamic_output_port?(name, type)
                !find_dynamic_output_ports(name, type).empty?
            end

            # Enumerates the input ports available on this task context. If no
            # block is given, returns the corresponding enumerator object.
            def each_input_port(&block)
                if block_given?
                    @ports.each do |p|
                        yield(p) if p.kind_of?(InputPort)
                    end
                    if superclass
                        superclass.each_input_port(&block) if superclass
                    end
                else
                    enum_for(:each_input_port)
                end
            end

            # A set of ports that will trigger this task when they get updated.
            attr_reader :event_ports

            # Declares that this task context is designed to be woken up when
            # new data is available on one of the given ports (or all already
            # defined ports if no names are given).
            def port_driven(*names)
                names = names.map { |n| n.to_s }
                relevant_ports = if names.empty? then all_ports.find_all { |p| p.kind_of?(InputPort) }
                                 else
                                     names.map do |n|
                                         obj = get_object(:ports, n)
                                         if !obj.kind_of?(InputPort)
                                             raise ArgumentError, "only read ports can be used as triggers for a task context"
                                         end
                                         obj
                                     end
                                 end

                @event_ports.concat(relevant_ports)
            end

            # Declares that this task context is designed to be woken up when
            # new data is available on a I/O file descriptor. The resulting task
            # must also use the fd_driven activity, which is done by default.
            #
            # The only thing you have to do in the implementation is therefore
            #
            #   task = task("MyDFDrivenTask").
            #       start
            #
            # To configure the activity, you will have to implement the
            # getFileDescriptor() method that is generated in the target class.
            def fd_driven
                default_activity "fd_driven"
                needs_configuration
            end

            # True if this task context's default activity is a FD-driven activity
            def fd_driven?
                default_activity.first == :fd_driven
            end
	    
	    # The set of task libraries from which we depend on, because of our 
	    # superclasses and implements
	    def used_task_libraries
		component.used_task_libraries.find_all do |tasklib|
		    tasklib.tasks.any? do |task|
			task.component == tasklib && implements?(task.name)
		    end
		end
	    end

            # Returns the set of types that are used to define this task
            # context, as an array of subclasses of Typelib::Type.
            def used_types
                (all_properties + all_methods + all_commands + all_ports).
                    map { |obj| obj.used_types }.
                    flatten.to_value_set.to_a
            end


            # Returns the set of imported toolkits that are needed to run this
            # task properly.
            def used_toolkits
                types = (all_properties + all_methods + all_commands + all_ports).
                    map { |obj| obj.used_types }.
                    flatten.to_value_set.to_a

                component.used_toolkits.find_all do |tk|
                    types.any? do |type|
                        tk.includes?(type.name)
                    end
                end.to_value_set
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


                fake_install_dir = File.join(component.base_dir, AUTOMATIC_AREA_NAME, component.name)
                FileUtils.mkdir_p fake_install_dir

                FileUtils.ln_sf File.join(component.base_dir, "tasks", "#{basename}.hpp"),
                    File.join(fake_install_dir, "#{basename}.hpp")
                FileUtils.ln_sf File.join(component.base_dir, AUTOMATIC_AREA_NAME, "tasks", "#{basename}Base.hpp"),
                    File.join(fake_install_dir, "#{basename}Base.hpp")

		self
	    end
	end
    end
end

