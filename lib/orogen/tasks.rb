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

        # Representation of a task's attribute or property
	class Property
            # The task on which this property is attached
            attr_reader :task
	    # The property name
	    attr_reader :name

	    # The property type, as a Typelib::Type object from the underlying
	    # component's type registry
	    attr_reader :type

            # The name of the type this property is using, for consistency with
            # the +type+ attribute
            def type_name; type.name end

            def used_types; [type] end

	    # The property's default value
	    attr_reader :default_value

            # The property default value, formatted for as a C++ value
            def cxx_default_value
                if type < Typelib::EnumType
                    type.namespace('::') + default_value.to_s
                else
                    default_value.inspect
                end
            end

	    # Create a new property with the given name, type and default value
	    def initialize(task, name, type, default_value)
                name = name.to_s
		if name !~ /^\w+$/
                    raise ArgumentError, "property names need to be valid C++ identifiers, i.e. contain only alphanumeric characters and _ (got #{name})"
		end

                type = task.component.find_type(type)
                Orocos.validate_toplevel_type(type)

		@name, @type, @default_value = name, type, default_value
	    end

            def pretty_print(pp)
                pp.text "#{name}:#{type.name}"
                if doc
                    pp.text ": #{doc}"
                end
            end

	    # call-seq:
	    #	doc new_doc -> self
            #	doc ->  current_doc
	    #
	    # Gets/sets a string describing this object
	    dsl_attribute(:doc) { |value| value.to_s }
	end
        Attribute = Property

        # Generic representation of ports. The actual ports are either
        # instance of InputPort or OutputPort
	class Port
	    # The port task
	    attr_reader :task
	    # The port name
	    attr_reader :name
	    # The port type
	    attr_reader :type
            # The port type name
            def type_name; type.name end

            def used_types; [type] end

            # True if the component supports only static connections on this
            # port, and false otherwise
            #
            # See #static for more details.
            def static?; !!@static end

            # Declares that this port can be connected/disconnected only when
            # the component is in a non-running state.
            #
            # The default is that the port is dynamic, i.e. can be
            # connected/disconnected regardless of the component's state.
            #
            # See also #dynamic
            def static; @static = true end

            # Declares that this port can be connected/disconnected while the
            # component is running. It is the opposite of #static.
            #
            # This is the default
            def dynamic; @static = false end

            def pretty_print(pp)
                pp.text "[#{self.kind_of?(InputPort) ? "in" : "out"}]#{name}:#{type_name}"
            end

	    def initialize(task, name, type)
                if !name.kind_of?(Regexp)
                    name = name.to_s
                    if name !~ /^\w+$/
                        raise ArgumentError, "port names need to be valid C++ identifiers, i.e. contain only alphanumeric characters and _ (got #{name})"
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
	    #	doc new_doc -> self
            #	doc ->  current_doc
	    #
	    # Gets/sets a string describing this object
	    dsl_attribute(:doc) { |value| value.to_s }
	end

        # Specification for an output port
        class OutputPort < Port
            def initialize(*args)
                super
                @sample_size  = 1
                @period = 1
                @burst_size   = 0
                @burst_period = 0
                @port_triggers = Set.new
            end

            # Returns the name of the Orocos class for this port (i.e.  one of
            # ReadDataPort, WriteDataPort, DataPort, ReadBufferPort, ...)
	    def orocos_class; "RTT::OutputPort" end

            attr_reader :burst_size
            attr_reader :burst_period

            # call-seq:
            #   sample_size new_size -> self
            #   sample_size -> size
            #
            # Sets and gets the sample size, i.e. how many data samples are
            # pushed at once to this port.
            dsl_attribute(:sample_size) { |value| Integer(value) }

	    # call-seq:
	    #	period new_period -> self
            #	period ->  current_period
	    #
            # Sets the period for this output port, in cycles. The port period
            # should be the minimal amount of execution cycles (calls to
            # updateHook) between two updates of this port.
            #
            # See #sample_size and #burst for other parameters describing the
            # behaviour of this port.
            #
            # The default is one.
            dsl_attribute(:period) { |value| Integer(value) }

	    # call-seq:
	    #	burst count, period -> self
	    #
            # Declares that a burst of data can occasionally be written to this
            # port. +count+ is the maximal number of samples that are pushed to
            # this port at once, and +period+ how often this burst can happen.
            #
            # If the perid is set to 0, then it is assumed that the bursts
            # happen 'every once in a while', i.e. that it can be assumed that
            # the event is rare enough.
            #
            # The default is no burst
            def burst(size, period = 1)
                @burst_size   = Integer(size)
                @burst_period = Integer(period)
                self
            end

            # The set of input ports that will cause a write on this output
            def port_triggers
                if @port_triggers.empty?
                    if !@triggered_on_update && (default = task.default_update_trigger)
                        [default.name].to_set
                    else
                        @port_triggers
                    end
                else
                    @port_triggers
                end
            end

            # call-seq:
            #   triggered_on input_port_name, input_port_name, ...
            #
            # Declares that this port will be written whenever a sample is
            # received on the given input ports. The default is to consider that
            # the port is written whenever updateHook() is called.
            #
            # You may want to call #triggered_on_update if the port will be
            # written for each call to updateHook too.
            def triggered_on(*input_ports)
                @port_triggers |= input_ports.to_set.map { |name| task.port(name) }
                self
            end

            # call-seq:
            #   triggered_on_update
            #
            # Declares that this port will be written for each call of the
            # updateHook(). It is the default if #triggered_on has not been
            # called.
            def triggered_on_update
                @triggered_on_update = true
                self
            end

            # True if the port will be written for the calls to updateHook()
            # that are triggered by the activity.
            #
            # See #triggered_on_update and #triggered_on
            def triggered_on_update?
                if !port_triggers.empty?
                    !!@triggered_on_update
                else
                    true
                end
            end
        end

        # Specification for an input port
        class InputPort < Port
            # Returns the name of the Orocos class for this port (i.e.  one of
            # ReadDataPort, WriteDataPort, DataPort, ReadBufferPort, ...)
	    def orocos_class; "RTT::InputPort" end

            attr_reader :required_connection_type

            def initialize(*args)
                super
                @required_connection_type = :data
            end

            # True if connections to this port must use a buffered.
            # In general, it means that the task's code check the return value
            # of read(), as in
            #
            #   if (_input.read(value))
            #   {
            #       // data is available, do something
            #   }
            def needs_buffered_connection; @required_connection_type = :buffer; self end

            # True if connections to this port must use a data policy.
            # 
            # This should not be useful in general
            def needs_data_connection; @required_connection_type = :data; self end

            # Returns true if the component requires connections to this port to
            # be reliable (i.e. non-lossy).
            #
            # See #needs_reliable_policy for more information
            def needs_reliable_connection?; @needs_reliable_connection end

            # Declares that the components requires a non-lossy policy
            #
            # This is different from #requires_buffered_connection as a data
            # policy could be used if the period of the connection's source is
            # much longer than the period of the connection's end (for
            # instance).
            def needs_reliable_connection; @needs_reliable_connection = true; self end
        end
        
        module DynamicPort
            def instanciate(name)
                m = dup
                m.instance_variable_set :@name, name
                m
            end
            def pretty_print(pp)
                pp.text "[dyn,#{self.class < InputPort ? "in" : "out"}]#{name}:#{type_name}"
            end
        end

        # Specification for a dynamic output port.
        #
        # Dynamic ports are not statically present, but will be created at
        # runtime. They are added by TaskContext#dynamic_output_port.
        class DynamicOutputPort < OutputPort
            include DynamicPort
        end
        
        # Specification for a dynamic input port.
        #
        # Dynamic ports are not statically present, but will be created at
        # runtime. They are added by TaskContext#dynamic_input_port.
        class DynamicInputPort < InputPort
            include DynamicPort
        end

        # Representation of a RTT operation. Instances of this object are
        # usually created through TaskContext#operation. The generated code will
        # expect the class implementation (user-visible part) to define one
        # method, to serve the call, with almost the same name that the method
        # itself.
        #
        # For instance, the following definition
        #   operation('MyMethod')
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
        # The name of the C++ method can be changed #method_name.
        #
        # For instance,
        #   operation('MyMethod').
        #     argument('x', 'double', 'the target X value').
        #     argument('y', 'double', 'the target Y value').
        #     method_name('move').
        #     returns('double')
        #
        # will require the user-visible part to define
        #   double move(double x, double y);
	class Operation
	    # The TaskContext instance this operation is part of
	    attr_reader :task
	    # The operation name
	    attr_reader :name
            # True if this operation runs its associated C++ method in caller
            # thread (default is false)
            #
            # See also #runs_in_caller_thread and #runs_in_callee_thread
            attr_reader :in_caller_thread

	    def initialize(task, name)
                name = name.to_s
		if name !~ /^\w+$/
                    raise ArgumentError, "#{self.class.name.downcase} names need to be valid C++ identifiers, i.e. contain only alphanumeric characters and _ (got #{name})"
		end

		@task = task
		@name = name
                @return_type = [nil, 'void']
		@arguments = []
                @in_caller_thread = false
		@method_name = self.name.dup
		method_name[0, 1] = method_name[0, 1].downcase
	    end

            # Declares that the C++ method associated with this operation should
            # be executed in the caller thread (default is callee thread)
            #
            # See also #runs_in_callee_thread and #in_caller_thread
            def runs_in_caller_thread
                @in_caller_thread = true
                self
            end

            # Declares that the C++ method associated with this operation should
            # be executed in the caller thread
            #
            # See also #runs_in_callee_thread and #in_caller_thread
            def runs_in_callee_thread
                @in_caller_thread = false
                self
            end

	    # call-seq:
	    #	doc new_doc -> self
            #	doc ->  current_doc
	    #
	    # Gets/sets a string describing this object
	    dsl_attribute(:doc) { |value| value.to_s }

	    # The set of arguments of this operation, as an array of [name, type,
	    # doc] elements. The +type+ objects are Typelib::Type instances.
            # 
            # See #argument
	    attr_reader :arguments

            # This version of find_type returns both a Typelib::Type object and
            # a normalized version for +name+. It does accept const and
            # reference qualifiers in +name+.
            def find_type(qualified_type)
                type_name = Orocos::Generation.unqualified_cxx_type(qualified_type)
		type      = task.component.find_type(type_name)
                Orocos.validate_toplevel_type(type)
                return type, qualified_type.gsub(type_name, type.cxx_name)
            end

            # Defines the next argument of this operation. +name+ is the argument
            # name and +type+ is either the type name as a string, or a
            # Typelib::Type object. In both cases, the required type must be
            # defined in the component, either because it is part of its own
            # typekit or because it has been imported by a
            # Component#load_typekit call.
            #
            # Note that Orocos::RTT does not support having more than 4
            # arguments for an operation, and trying that will therefore raise an
            # error
	    def argument(name, qualified_type, doc = "")
                if arguments.size == 4
                    raise ArgumentError, "Orocos does not support having more than 4 arguments for an operation"
                end

                type, qualified_type = find_type(qualified_type)
		arguments << [name, type, doc, qualified_type]
		self
	    end

            # Shortcut for #arg
            def arg(*args, &block)
                argument(*args, &block)
            end

            # Returns the set of types that this operation uses, as a
            # ValueSet of Typelib::Type classes.
            def used_types
                [return_type.first].compact + arguments.map { |_, t, _| t }
            end

	    # Returns the argument part of the C++ signature for this callable
	    def argument_signature(with_names = true)
		arglist = arguments.map do |name, type, doc, qualified_type|
                    # Auto-add const-ref for non-trivial types
                    arg =
                        if type.cxx_name == qualified_type && !(type < Typelib::NumericType)
                            "#{type.cxx_name} const &"
                        else
                            qualified_type
                        end

		    if with_names then "#{arg} #{name}"
		    else arg
		    end
		end

		"(" << arglist.join(", ") << ")"
	    end

	    # The return type of this operation, as a [type_object,
            # qualified_cxx_type] pair.
            #
            # See #returns
	    attr_reader :return_type

            # Sets the return type for this operation. +type+ can either be the
            # type name or a Typelib::Type object. In both cases, the required
            # type must be defined in the component, either because it is part
            # of its own typekit or because it has been imported by a
            # Component#load_typekit call.
	    def returns(type)
                @return_type =
                    if type then find_type(type)
                    else [nil, 'void']
                    end

		self
	    end

            # Returns true if this operation's signature is not void
            def has_return_value?
                !!@return_type.first
            end

            # Returns the C++ signature for this operation. Used in code
            # generation only.
	    def signature(with_names = true)
		result = return_type.last.dup
                if with_names
                    result << " " <<
                        if block_given? then yield
                        else method_name
                        end
                end
		result << argument_signature(with_names)
	    end

            def pretty_print(pp)
                pp.text signature(true)
            end

	    # call-seq:
	    #	method_name new_name -> self
            #	method_name -> current_name
	    #
            # Gets or sets the name of the C++ method which is to be called to
            # serve this operation. It default to the method name with the
            # first character set to lowercase (MyMethod becomes myMethod).
	    dsl_attribute(:method_name) { |value| value.to_s }
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
	    # The oroGen project this task is part of
	    attr_reader :project

            # for backward compatibility reasons
            def component # :nodoc:
                project
            end

	    # The task name
	    attr_reader :name
            # The subclass of TaskContext which should be used to define this
            # class
            attr_reader :superclass
            # A set of classes the TaskContext has to implement as well
            attr_reader :implemented_classes
            # A set of Port objects that can be created at runtime
            attr_reader :dynamic_ports

            def self.enumerate_inherited_set(each_name, attribute_name = each_name) # :nodoc:
                class_eval <<-EOD
                def all_#{attribute_name}; each_#{each_name}.to_a end
                def self_#{attribute_name}; @#{attribute_name} end
                def each_#{each_name}(only_self = false, &block)
                    if block_given?
                        if !only_self && superclass
                            superclass.each_#{each_name}(false, &block)
                        end
                        @#{attribute_name}.each(&block)
                    else
                        enum_for(:each_#{each_name}, only_self)
                    end
                end
                EOD
            end

            def to_s; "#<#<Orocos::Generation::TaskContext>: #{name}>" end
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
                ancestor_names = ancestors.map(&:name)
                class_name == name ||
                    ancestor_names.include?(name)
            end

            ##
            # :method: required_activity?
            #
            # True if the current value of default_activity is actually
            # required by the task context implementation
            attr_predicate :required_activity?, true

            ##
            # :method: required_activity
            # :call-seq:
            #   required_activity 'activity_type', *args
            #
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

            ##
            # :method: default_activity
            # :call-seq:
            #   default_activity 'avtivity_type', *args
            #
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
	    def initialize(project, name)
                if name == project.name
                    raise ArgumentError, "tasks and projects must not have the same name"
                elsif name !~ /^(\w+::)*\w+$/
		    raise ArgumentError, "task names need to be valid C++ identifiers, i.e. contain only alphanumeric characters and _ (got #{name})"
		end

                @project  = project
                if name != "RTT::TaskContext"
                    @superclass = project.default_task_superclass
                end
                @implemented_classes = []
		@name = name
                # This is an array, as we don't want to have it reordered
                # unnecessarily
                @states = Array.new
                default_activity 'triggered'

		@properties = Array.new
		@attributes = Array.new
		@operations = Array.new
		@ports	    = Array.new
                @dynamic_ports = Array.new
                @event_ports = Array.new
                @initial_state = 'Stopped'

                @fixed_initial_state = false
                @needs_configuration = false
	    end

            # Returns the task context models that are in this model's ancestry
            def ancestors
                m = self
                result = [self]
                while m = m.superclass
                    result << m
                end
                result
            end

            def pretty_print_interface(pp, name, set)
                if set.empty?
                    pp.text "No #{name.downcase}"
                else
                    pp.text name
                    pp.nest(2) do
                        set.each do |element|
                            pp.breakable
                            element.pretty_print(pp)
                        end
                    end
                end
                pp.breakable
            end

	    def pretty_print(pp)
		pp.text "------- #{name} ------"
                pp.breakable

                ports = each_port.to_a + each_dynamic_port.to_a
                pretty_print_interface(pp, "Ports", ports)
                pretty_print_interface(pp, "Properties", each_property.to_a)
                pretty_print_interface(pp, "Attributes", each_attribute.to_a)
                pretty_print_interface(pp, "Operations", each_operation.to_a)
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
                # Check if that name is a method name in orocos.rb as well ...
                # To warn about name clashes
                if @orocos_rb.nil?
                    begin
                        require 'orocos'
                        @orocos_rb = true
                    rescue LoadError
                        @orocos_rb = false
                    end
                end

                if name.to_str != 'state' && @orocos_rb && !component.kind_of?(ImportedProject)
                    if Orocos::TaskContext.instance_methods.find { |n| n.to_s == name.to_str }
                        STDERR.puts "WARN: #{name} is a method name used in orocos.rb"
                        STDERR.puts "WARN:   if you keep that name, you will not be able to use shortcut access in orocos.rb"
                        STDERR.puts "WARN:   for instance, for a property, you will have to do"
                        STDERR.puts "WARN:      value = my_task.property('#{name}').read(new_value)"
                        STDERR.puts "WARN:   instead of the shorter and clearer"
                        STDERR.puts "WARN:      value = my_task.#{name}"
                    end
                end

		set = send("all_#{set_name}")
		if set.find { |o| o.name == name }
		    raise ArgumentError, "there is already a #{name} in #{set_name}"
		end
	    end
	    private :check_uniqueness

            # Add in +self+ the ports of +other_model+ that don't exist.
            #
            # Raises ArgumentError if +other_model+ has ports whose name is used
            # in +self+, but for which the definition is different.
            def merge_ports_from(other_model)
                other_model.each_port do |p|
                    begin
                        self_port = port(p.name)
                        if (self_port.class != p.class || self_port.type != p.type)
                            raise ArgumentError, "cannot merge as the output port #{self_port.name} have different meanings"
                        end
                    rescue ArgumentError
                        @ports << p
                    end
                end
            end

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


            # Create a new attribute with the given name, type and default value
            # for this task. This returns an Attribute instance representing
            # the new attribute, whose methods can be used to configure it
            # further. +type+ is the type name for that attribute.  It
            # can be either in Typelib notation (/std/string) or in C++
            # notation (std::string). This type must be defined either by the
            # component's own typekit, or by typekits imported with
            # Component#load_typekit.
            #
            # The generated task context will have a <tt>_[attribute name]</tt>
            # attribute of class RTT::Attribute<type>.
            #
            # For instance, the following definition
            #   attribute('device_name', '/std/string/, '').
            #       doc 'the device name to connect to'
            #
            # Will generate a task context with a <tt>_device_name</tt>
            # attribute of type RTT::Attribute<std::string>.
	    def attribute(name, type, default_value = nil)
                name = name.to_s if name.respond_to?(:to_sym)
		check_uniqueness :attributes, name
		type = component.find_type(type)

		@attributes << Attribute.new(self, name, type, default_value)
		@attributes.last
	    end

            # Create a new property with the given name, type and default value
            # for this task. This returns the Property instance representing
            # the new property, whose methods can be used to configure the
            # property further. +type+ is the type name for that property.  It
            # can be either in Typelib notation (/std/string) or in C++
            # notation (std::string). This type must be defined either by the
            # component's own typekit, or by typekits imported with
            # Component#load_typekit.
            #
            # The generated task context will have a <tt>_[property name]</tt>
            # property of class RTT::Property<type>.
            #
            # For instance, the following definition
            #   property('device_name', '/std/string/, '').
            #       doc 'the device name to connect to'
            #
            # Will generate a task context with a <tt>_device_name</tt>
            # attribute of type RTT::Property<std::string>.
	    def property(name, type, default_value = nil)
                name = Generation.verify_valid_identifier(name)
		check_uniqueness :properties, name
		type = component.find_type(type)

		@properties << Property.new(self, name, type, default_value)
		@properties.last
            rescue Typelib::NotFound
                raise ConfigError, "type #{type} is not declared"
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
                    elsif state_port.type != component.find_type("/int")
                        raise ArgumentError, 
                            "there is already an output port called 'state', but it is not of type 'int' (found #{state_port.typename}"
                    end
                else
                    output_port 'state', '/int'
                end

                # Force typekit generation. The typekit code will take care of
                # generating the state enumeration type for us
                component.typekit(true)

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

            STATE_TYPES = [ :toplevel, :runtime, :error, :fatal, :exception ]

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
            # See also #each_error_state, #each_exception_state, #each_fatal_state and #each_state

            ##
            # :method: each_error_state
            #
            # Enumerates all error states defined for this task context
            #
            # See also #each_runtime_state, #each_exception_state, #each_fatal_state, and #each_state

            ##
            # :method: each_exception_state
            #
            # Enumerates all error states defined for this task context
            #
            # See also #each_runtime_state, #each_fatal_state, #each_error_state and #each_state

            ##
            # :method: each_fatal_state
            #
            # Enumerates all error states defined for this task context
            #
            # See also #each_runtime_state, #each_exception_state, #each_error_state and #each_state

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
            #   states -> set of states
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
            # See #error_states, #exception_states, #each_state, #each_runtime_state
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
            # See #runtime_states, #exception_states, #each_state, #each_error_state
            def error_states(*state_names)
                state_names.each do |name|
                    define_state(name, :error)
                end
            end

            # Declares a certain number of exception states
            #
            # This method will do nothing if it defines a state that is already
            # defined by one of the superclasses.
            #
            # See #runtime_states, #fatal_states, #error_states, #each_state, #each_error_state
            def exception_states(*state_names)
                state_names.each do |name|
                    define_state(name, :exception)
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
            # is actually defined on the project's typekit.
            def shared_ptr(name)
                base_type = component.find_type(name)
                full_name = "/boost/shared_ptr<#{base_type.name}>"
                begin
                    component.find_type(full_name)
                rescue Typelib::NotFound
                    component.typekit { shared_ptr(name) }
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
            # is actually defined on the project's typekit.
            def ro_ptr(name)
                base_type =
                    begin
                        component.find_type(name)
                    rescue Typelib::NotFound
                        raise ArgumentError, "type #{name} is not available"
                    end

                full_name = "/RTT/extras/ReadOnlyPointer<#{base_type.name}>"
                begin
                    component.find_type(full_name)
                rescue Typelib::NotFound
                    component.typekit { ro_ptr(name) }
                    component.find_type(full_name)
                end
            end

            # Create a new operation with the given name. Use the returned
            # Operation object to configure it further
	    #
            # In Orocos, an operation publishes a C++ method to the component
            # interface. The operation can then be called by other components
            # remotely or locally, and synchronoulsy as well as asynchronously.
	    def operation(name)
                name = Generation.verify_valid_identifier(name)
		check_uniqueness(:operations, name)
		@operations << Operation.new(self, name)
		@operations.last
	    end

            # Interface for RTT 1.x methods
            #
            # This raises NotImplementedError with a message asking to convert
            # to RTT2 operations
            def method(name)
                raise NotImplementedError, "RTT 1.x methods must be replaced by RTT 2.x operations. Use #operation"
            end

            # Interface for RTT 1.x commands
            #
            # This raises NotImplementedError with a message asking to convert
            # to RTT2 operations
            def command(name)
                raise NotImplementedError, "RTT 1.x commands must be replaced by RTT 2.x operations. Use #operation"
            end

            ##
            # :method: each_dynamic_port
            # :call-seq:
            #   each_dynamic_port(only_self = false) { |port| }
            #
            # Yields all dynamic ports that are defined on this task context.

            ##
            # :method: all_dynamic_ports
            # :call-seq:
            #   all_dynamic_ports -> set_of_ports
            #
            # Returns the set of all dynamic ports that are defined on this task
            # context

            ##
            # :method: self_dynamic_ports
            # :call-seq:
            #   self_dynamic_ports -> set_of_ports
            #
            # Returns the set of dynamic ports that are added at this level of
            # the model hierarchy. I.e. ports that are defined on this task
            # context, but not on its parent models.

            enumerate_inherited_set("dynamic_port", "dynamic_ports")

            ##
            # :method: each_port
            # :call-seq:
            #   each_port(only_self = false) { |port| }
            #
            # Yields all static ports that are defined on this task context.

            ##
            # :method: all_ports
            # :call-seq:
            #   all_ports -> set_of_ports
            #
            # Returns the set of all static ports that are defined on this task
            # context

            ##
            # :method: self_ports
            # :call-seq:
            #   self_ports -> set_of_ports
            #
            # Returns the set of static ports that are added at this level of
            # the model hierarchy. I.e. ports that are defined on this task
            # context, but not on its parent models.

            enumerate_inherited_set("port", "ports")

            ##
            # :method: each_attribute
            # :call-seq:
            #   each_attribute(only_self = false) { |attribute| }
            #
            # Yields all attributes that are defined on this task context.

            ##
            # :method: all_attributes
            # :call-seq:
            #   all_attributes -> set_of_attributes
            #
            # Returns the set of all attributes that are defined on this task
            # context

            ##
            # :method: self_attributes
            # :call-seq:
            #   self_attributes -> set_of_attributes
            #
            # Returns the set of attributes that are added at this level of the
            # model hierarchy. I.e. attributes that are defined on this task
            # context, but not on its parent models.

            enumerate_inherited_set("attribute", "attributes")

            ##
            # :method: each_property
            # :call-seq:
            #   each_property(only_self = false) { |property| }
            #
            # Yields all properties that are defined on this task context.

            ##
            # :method: all_properties
            # :call-seq:
            #   all_properties -> set_of_properties
            #
            # Returns the set of all properties that are defined on this task
            # context

            ##
            # :method: self_properties
            # :call-seq:
            #   self_properties -> set_of_properties
            #
            # Returns the set of properties that are added at this level of the
            # model hierarchy. I.e. properties that are defined on this task
            # context, but not on its parent models.

            enumerate_inherited_set("property", "properties")

            ##
            # :method: each_command
            # :call-seq:
            #   each_command(only_self = false) { |command| }
            #
            # Yields all commands that are defined on this task context.

            ##
            # :method: all_commands
            #
            # :call-seq:
            #   all_commands -> set_of_commands
            #
            # Returns the set of all commands that are defined on this task
            # context

            ##
            # :method: self_commands
            # :call-seq:
            #   self_commands -> set_of_commands
            #
            # Returns the set of commands that are added at this level of the
            # model hierarchy. I.e. commands that are either newly defined on
            # this task context, or overload commands from the parent models.

            enumerate_inherited_set("command", "commands")

            ##
            # :operation: each_operation
            # :call-seq:
            #   each_operation(only_self = false) { |operation| ... }
            #
            # Yields all operations that are defined on this task context.

            ##
            # :operation: all_operations
            # :call-seq:
            #   all_operations -> set_of_operations
            #
            # Returns the set of all operations that are defined on this task
            # context

            ##
            # :operation: self_operations
            # :call-seq:
            #   self_operations -> set_of_operations
            #
            # Returns the set of operations that are added at this level of the
            # model hierarchy. I.e. operations that are either newly defined on
            # this task context, or overload operations from the parent models.

            enumerate_inherited_set("operation", "operations")

            # Operations that are added by this task context (i.e. operations that are
            # defined there but are not present in the superclass)
            def new_operations
                super_names = superclass.all_operations.map(&:name).to_set
                @operations.find_all do |t|
                    !super_names.include?(t)
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
                name = Generation.verify_valid_identifier(name)
		check_uniqueness(:ports, name)
                @ports << OutputPort.new(self, name, type)
                @ports.last
            rescue Typelib::NotFound
                raise ConfigError, "type #{type} is not declared"
	    end

            # Enumerates the output ports available on this task context. If no
            # block is given, returns the corresponding enumerator object.
            def each_output_port(&block)
                each_port.
                    find_all { |p| p.kind_of?(OutputPort) }.
                    each(&block)
            end

            # Returns the port named +name+ or raises ArgumentError if no such
            # port exists
            def port(name)
                if p = each_port.find { |p| p.name == name }
                    p
                else raise ArgumentError, "#{self} has no port named '#{name}'"
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
                name = Generation.verify_valid_identifier(name)
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
                dynamic_ports << DynamicInputPort.new(self, name, type)
                dynamic_ports.last
            end

            # call-seq:
            #   dynamic_output_port name_regex, typename
            #
            # Declares that a port whose name matches name_regex can be declared
            # at runtime, with the type. This is not used by orogen himself, but
            # can be used by potential users of the orogen specification.
            def dynamic_output_port(name, type)
                dynamic_ports << DynamicOutputPort.new(self, name, type)
                dynamic_ports.last
            end

            # Returns true if there is a dynamic port definition that matches
            # the given name and type pair.
            #
            # If +type+ is nil, the type is ignored in the matching.
            def dynamic_port?(name, type)
                dynamic_input_port?(name, type) || dynamic_output_port?(name, type)
            end

            # Returns the set of dynamic input port definitions that match the
            # given name and type pair. If +type+ is nil, the type is ignored in
            # the matching.
            def find_dynamic_input_ports(name, type)
                dynamic_ports.find_all { |p| p.kind_of?(InputPort) && (!type || p.type == component.find_type(type)) && p.name === name }
            end

            # Returns true if there is an input port definition that match the
            # given name and type pair. If +type+ is nil, the type is ignored in
            # the matching.
            def dynamic_input_port?(name, type = nil)
                !find_dynamic_input_ports(name, type).empty?
            end

            # Returns the set of dynamic output port definitions that match the
            # given name and type pair. If +type+ is nil, the type is ignored in
            # the matching.
            def find_dynamic_output_ports(name, type)
                dynamic_ports.find_all { |p| p.kind_of?(OutputPort) && (!type || p.type == component.find_type(type)) && p.name === name }
            end

            # Returns true if an output port of the given name and type could be
            # created at runtime.
            def dynamic_output_port?(name, type = nil)
                !find_dynamic_output_ports(name, type).empty?
            end

            # Enumerates the input ports available on this task context. If no
            # block is given, returns the corresponding enumerator object.
            def each_input_port(&block)
                each_port.
                    find_all { |p| p.kind_of?(InputPort) }.
                    each(&block)
            end

            # Default input port that triggers writing on outputs.
            #
            # See #updates_on
            attr_reader :default_update_trigger

            # Declares that all input ports for which no triggering mechanism is
            # specified, either by using #triggered_on or by explicitely calling
            # #triggered_on_update are actually triggered by the given input
            # port
            def updates_on(name)
                @default_update_trigger = port(name.to_str)
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
	    
	    # The set of task libraries that are required by this task context
            #
            # This is the set of task libraries that implement our superclasses
	    def used_task_libraries
		component.used_task_libraries.find_all do |tasklib|
		    tasklib.self_tasks.any? do |task|
			implements?(task.name)
		    end
		end
	    end

            # Returns the set of types that are used to define this task
            # context, as an array of subclasses of Typelib::Type.
            def interface_types
                (all_properties + all_operations + all_ports).
                    map { |obj| obj.used_types }.
                    flatten.to_value_set.to_a
            end


            # Returns the set of typekits that define the types used in this
            # task's interface. They are required at compile and link time
            # because of the explicit instanciation of interface templates
            # (ports, ...)
            def used_typekits
                types = interface_types
                component.used_typekits.find_all do |tk|
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

            # Generate a graphviz fragment to represent this task
            def to_dot
                html_escape = lambda { |s| s.gsub(/</, "&lt;").gsub(/>/, "&gt;") }
                html_table  = lambda do |title, lines|
                    label  = "<TABLE BORDER=\"0\" CELLBORDER=\"1\" CELLSPACING=\"0\">\n"
                    label << "  <TR><TD>#{title}</TD></TR>\n"
                    label << "  <TR><TD>\n"
                    label << lines.join("<BR/>\n")
                    label << "  </TD></TR>\n"
                    label << "</TABLE>"
                end
                    
                result = ""
                result << "  node [shape=none,margin=0,height=.1];"

                label = ""
                label << "<TABLE BORDER=\"0\" CELLBORDER=\"0\" CELLSPACING=\"0\">\n"
                label << "  <TR><TD>#{name}</TD></TR>"

                properties = all_properties.
                    map { |p| "#{p.name} [#{html_escape[p.type_name]}]" }
                if !properties.empty?
                    label << "  <TR><TD>#{html_table["Properties", properties]}</TD></TR>"
                end


                input_ports = all_ports.
                    find_all { |p| p.kind_of?(InputPort) }.
                    map { |p| "#{p.name} [#{html_escape[p.type_name]}]" }
                if !input_ports.empty?
                    label << "  <TR><TD>#{html_table["Input ports", input_ports]}</TD></TR>"
                end

                output_ports =all_ports.
                    find_all { |p| p.kind_of?(OutputPort) }.
                    map { |p| "#{p.name} [#{html_escape[p.type_name]}]" }
                if !output_ports.empty?
                    label << "  <TR><TD>#{html_table["Output ports", output_ports]}</TD></TR>"
                end

                label << "</TABLE>"
                result << "  t#{object_id} [label=<#{label}>]"
                result
            end
	end
    end
end

