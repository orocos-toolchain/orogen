module OroGen
    module Spec
        ACTIVITY_TYPES = {
            :fd_driven    => 'FileDescriptorActivity',
            :irq_driven   => 'IRQActivity',
            :slave        => 'SlaveActivity',
            :periodic     => 'PeriodicActivity',
            :triggered    => 'NonPeriodicActivity',
            :sequential   => 'SequentialActivity'
        }

        # Base class for all task model extensions. It provides a few useful
        # common methods
        class TaskModelExtension
            attr_reader :name

            def initialize(name = nil)
                @name = name
            end

            def supercall(default, m, *args, &block)
                if self.name && @super_ext || (@super_ext = task.superclass.find_extension(self.name))
                    @super_ext.send(m, *args, &block)
                else
                    default
                end
            end

            # Called at registration time so that the extension can apply some default stuff
            #
            # @param [TaskContext]
            def registered_on(task_context)
            end
        end

        # Model of a task context, i.e. a task context interface
        #
        # The corresponding code generation support is done in
        # {Gen::RTT_CPP::TaskContextGeneration}
	class TaskContext
	    # The oroGen project this task is part of
	    attr_reader :project

            ## :method: doc
            # :call-seq:
            #   doc => string
            #   doc "documentation string"
            #   
            # Gets or sets the documentation string for this task context
            dsl_attribute :doc

            class << self
                # @return [Symbol] set of method names that should be called on
                #   the newly created task at creation time. This is meant to
                #   register some default extensions automatically
                attr_reader :default_extensions
                attr_reader :extensions_disabled

                def disable_default_extensions
                    @extensions_disabled = true
                end

                def enable_default_extensions
                    @extensions_disabled = false
                end

                def apply_default_extensions(task_context)
                    if !extensions_disabled
                        default_extensions.each do |ext|
                            task_context.send(ext)
                        end
                    end
                end
            end
            @extensions_disabled = false
            @default_extensions = Array.new

            enumerate_inherited_map 'default_extension', 'default_extensions'

            # Set of extensions registered for this task
            #
            # Extensions are named objects of arbitrary type that can be used to
            # extend the task context model. If they contain a #generate method,
            # this method is also called to add some code generation elements
            #
            # This is an array (not a hash) so that it keeps order
            attr_reader :extensions

            # True if an extension with the given name has been registered
            def has_extension?(name, with_subclasses = true); !!find_extension(name, with_subclasses) end

            # Registers an extension with the given name. Raises ArgumentError
            # if there is already one.
            def register_extension(obj)
                if (old = find_extension(obj.name, false)) && old != obj
                    raise ArgumentError, "there is already an extension called #{name}: #{old}"
                else
                    extensions << obj
                    obj.registered_on(self)
                end
            end

            # Enumerates the extensions registered on this task model, as (name,
            # extension_object) pairs
            def each_extension(with_subclasses = true, &block)
                if !block_given?
                    return enum_for(:each_extension, with_subclasses)
                end

                seen = Set.new
                klass = self
                while klass
                    klass.extensions.each do |ext|
                        if !seen.include?(ext.name)
                            seen << ext.name
                            yield(ext)
                        end
                    end
                    klass = klass.superclass
                end
            end

            # Returns the extension named +name+, or nil if there is none
            def find_extension(name, with_subclasses = true)
                if result = extensions.find { |ext| ext.name == name }
                    result
                elsif with_subclasses && superclass
                    superclass.find_extension(name, true)
                end
            end

            # Returns the extension named +name+, or raises ArgumentError if
            # none is registered with that name
            def extension(name, with_subclasses = true)
                if ext = find_extension(name, with_subclasses)
                    ext
                else raise ArgumentError, "no extension registered under the name '#{name}'"
                end
            end

            def to_s; "#<OroGen::Spec::TaskContext: #{name}>" end
            def inspect; to_s end

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
            # scope of the enclosing Project object -- i.e. either defined in
            # it, or imported by a Project#using_task_library call.
            def subclasses(task_context)
                if task_context.respond_to?(:to_str)
                    if @superclass != project.default_task_superclass
                        raise OroGen::ConfigError, "#{@name} tries to subclass #{task_context} "+
                            "while there is already #{@superclass.name}"
                    end
                    @superclass = project.task_model_from_name task_context
                else
                    @superclass = task_context
                end
                @default_activity  = @superclass.default_activity.dup
                @required_activity = @superclass.required_activity?
                if !superclass
                    raise ArgumentError, "no such task context #{task_context}"
                end
            end

            # Declares that this task context is a root model and
            # does not have a superclass
            def root_model
                @superclass = nil
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
                self.name == name ||
                    ancestor_names.include?(name)
            end

            # A documentation string for this task context. It is used by
            # display tools and documentation generation.
            dsl_attribute 'doc' do |str|
                str.to_str
            end

            ##
            # :method: worstcase_processing_time
            # :call-seq:
            #   worstcase_processing_time => value
            #   worstcase_processing_time value
            #
            # Sets or gets the worst-case computation time (i.e. time spent in
            # 'update') for this task context. This should usually not be set in
            # the oroGen file, but in the supervision/deployment code, since the
            # actual computation time will depend on the system.
            #
            # The time is given in seconds
            dsl_attribute :worstcase_processing_time do |value|
                Float(value)
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
                    send(type.to_sym, *args)
                else
                    default_activity type, *args
                end
                self.required_activity = true
            end

            ##
            # :method: default_activity
            # :call-seq:
            #   default_activity 'activity_type', *args
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

            # Declares that this task should be deployed using a default
            # periodic activity, with the given period
            def periodic(period)
                default_activity :periodic, period
            end
            
            # True if this task context is defined by one of our dependencies.
            attr_predicate :external_definition?, true

	    # Create a new task context in the given project and with
	    # the given name. If a block is given, it is evaluated
	    # in the context of the newly created TaskContext object.
	    #
	    # TaskContext objects should not be created directly. You should
	    # use {Project#task_context} for that.
	    def initialize(project, name = nil)
                @project  = project

                @superclass = project.default_task_superclass
                @implemented_classes = []
		@name = name
                # This is an array, as we don't want to have it reordered
                # unnecessarily
                @states = Array.new
                default_activity 'triggered'

		@properties = Hash.new
		@attributes = Hash.new
		@operations = Hash.new
		@output_ports = Hash.new
		@input_ports  = Hash.new
                @dynamic_ports = Array.new
                @event_ports = Hash.new
                @initial_state = 'Stopped'
                @default_extensions = Array.new
                @fixed_initial_state = false
                @needs_configuration = false

                ## WARN: this must be kept an array so that the generation order
                ## WARN: is deterministic
                @extensions = Array.new

                super()

                if block_given?
                    instance_eval(&proc)
                end
	    end

            def initialize_copy(from)
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
                        set = set.to_a.sort_by { |p| p.name.to_s }
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
                if doc
                    first_line = true
                    doc.split("\n").each do |line|
                        pp.breakable if !first_line
                        first_line = false
                        pp.text "# #{line}"
                    end
                    pp.breakable
                    pp.text "# "
                else
                    pp.text "no documentation defined for this task context model"
                end
                pp.breakable
                pp.text "subclass of #{superclass.name} (the superclass elements are displayed below)"
                pp.breakable
                triggers = all_event_ports
                if !triggers.empty?
                    pp.text "Triggered on input: #{triggers.map(&:name).join(", ")}"
                    pp.breakable
                end
                if needs_configuration?
                    pp.text "Needs configuration"
                else
                    pp.text "Does NOT need configuration"
                end
                pp.breakable

                pretty_print_interface(pp, "Ports", each_port.to_a)
                pretty_print_interface(pp, "Dynamic Ports", each_dynamic_port.to_a)
                pretty_print_interface(pp, "Properties", each_property.to_a)
                pretty_print_interface(pp, "Attributes", each_attribute.to_a)
                pretty_print_interface(pp, "Operations", each_operation.to_a)

                extensions.each do |ext|
                    pp.breakable
                    pp.text "Extension: #{ext.name}"
                    pp.nest(2) do
                        pp.breakable
                        ext.pretty_print(pp)
                    end
                end
	    end

	    # Raises ArgumentError if an object named +name+ is already present
	    # in the set attribute +set_name+. 
	    #
	    # This is an internal helper method
	    def check_uniqueness(name) # :nodoc:
                obj = find_input_port(name) ||
                    find_output_port(name) ||
                    find_operation(name) ||
                    find_property(name) ||
                    find_attribute(name)

		if obj
		    raise ArgumentError, "#{name} is already used in the interface of #{self.name}, as a #{obj.class}"
		end
	    end

            # Add in +self+ the ports of +other_model+ that don't exist.
            #
            # Raises ArgumentError if +other_model+ has ports whose name is used
            # in +self+, but for which the definition is different.
            def merge_ports_from(other_model, name_mappings = Hash.new)
                other_model.each_port do |p|
                    if target_name = name_mappings[p.name]
                        p = p.dup
                        p.instance_variable_set(:@name, target_name.to_str)
                    end

                    if has_port?(p.name)
                        self_port = find_port(p.name)
                        if self_port.class != p.class
                            raise ArgumentError, "cannot merge as #{self_port.name} is a #{self_port.class} in #{self} and a #{p.class} in #{other_model}"
                        elsif self_port.type != p.type
                            raise ArgumentError, "cannot merge as #{self_port.name} is of type #{self_port.type} in #{self} and of type #{p.type} in #{other_model}"
                        end
                    elsif p.kind_of?(OutputPort)
                        @output_ports[p.name] = p
                    elsif p.kind_of?(InputPort)
                        @input_ports[p.name] = p
                    end
                end
                other_model.each_dynamic_port do |p|
                    existing = each_dynamic_port.find_all do |self_p|
                        p.name == self_p.name
                    end
                    if !existing.any? { |self_p| self_p.type == p.type }
                        self.dynamic_ports << p.dup
                    end
                end
            end

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
                if superclass && superclass.fixed_initial_state? && !superclass.needs_configuration?
                    raise ArgumentError, "cannot change the start state of this task context: the superclass #{superclass.name} does not allow it"
                elsif fixed_initial_state? && !needs_configuration?
                    raise ArgumentError, "cannot change the start state of this task context: #fixed_initial_state has been specified for it"
                end
                @needs_configuration = true
            end

            # Create a new attribute with the given name, type and default value
            # for this task. This returns an Attribute instance representing
            # the new attribute, whose methods can be used to configure it
            # further. +type+ is the type name for that attribute.
            #
            # @example
            #   attribute('device_name', '/std/string/, '').
            #       doc 'the device name to connect to'
	    def attribute(name, type, default_value = nil)
		@attributes[name] = att = configuration_object(Attribute, name, type, default_value)
                Spec.load_documentation(att, /attribute/)
                att
	    end

            def configuration_object(klass, name, type, default_value)
                name = OroGen.verify_valid_identifier(name)
                check_uniqueness(name)

                begin
                    type = project.find_interface_type(type)
                rescue Typelib::NotFound => e
                    raise ConfigError, "invalid type #{type}: #{e.message}", e.backtrace
                end

                if default_value
                    accepted = [Numeric, Symbol, String, TrueClass, FalseClass].
                        any? { |valid_klass| default_value.kind_of?(valid_klass) }
                    if !accepted
                        raise ArgumentError, "default values for #{klass.name.downcase} can be specified only for simple types (numeric, string and boolean)"
                    end
                    begin
                        Typelib.from_ruby(default_value, type)
                    rescue Typelib::UnknownConversionRequested => e
                        raise ArgumentError, e.message, e.backtrace
                    end
                end

		klass.new(self, name, type, default_value)
            end

            # Create a new property with the given name, type and default value
            # for this task. This returns the Property instance representing
            # the new property, whose methods can be used to configure the
            # property further. +type+ is the type name for that property.  It
            # can be either in Typelib notation (/std/string) or in C++
            # notation (std::string). This type must be defined either by the
            # project's own typekit, or by typekits imported with
            # Project#load_typekit.
            #
            # @example
            #   property('device_name', '/std/string/, '').
            #       doc 'the device name to connect to'
	    def property(name, type, default_value = nil)
		@properties[name] = prop = configuration_object(Property, name, type, default_value)
                Spec.load_documentation(prop, /property/)
                prop
	    end

            # True if this task has a property with that name
            def has_property?(name)
                !!find_property(name)
            end

            # Make an existing property dynamic
            #
            # @return [Property] the property object
            def make_property_dynamic(name)
                prop = find_property(name)
                if !prop
                    raise ArgumentError, "The requested property " + name + " could not be found"
                end
                property = @properties[name] = prop.dup
                property.task = self
                property.dynamic
                property
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
                state_port = find_port("state")
                if state_port
                    if state_port.kind_of?(InputPort)
                        raise ArgumentError, 
                            "there is already an input port called 'state', cannot enable extended state support"
                    elsif state_port.type != project.find_type("/int32_t")
                        raise ArgumentError, 
                            "there is already an output port called 'state', but it is not of type 'int' (found #{state_port.type_name}"
                    end
                else
                    output_port('state', '/int32_t').
                        triggered_once_per_update
                end

                # Force typekit generation. The typekit code will take care of
                # generating the state enumeration type for us
                project.typekit(true)

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

            # Declares a certain number of reports
            #
            # This method will do nothing if it defines a report that is already
            # defined by one of the superclasses.
            def reports(*state_names)
                runtime_states(*state_names)
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

            # Create a new operation with the given name. Use the returned
            # Operation object to configure it further
	    def operation(name)
                name = OroGen.verify_valid_identifier(name)
		@operations[name] = op = Operation.new(self, name)
                Spec.load_documentation(op, /operation/)
                op
	    end

            # Defines an operation whose implementation is in the Base class
            # (i.e. "hidden" from the user)
            def hidden_operation(name, body)
                op = operation(name)
                op.hidden = true
                op.base_body = body
                op
            end

            ##
            # :method: each_dynamic_input_port
            # :call-seq:
            #   each_dynamic_port(only_self = false) { |port| }
            #
            # Yields all dynamic input ports that are defined on this task context.
            def each_dynamic_input_port(only_self = false)
                return enum_for(:each_dynamic_input_port, only_self) if !block_given?
                each_dynamic_port do |port|
                    if port.kind_of?(InputPort)
                        yield(port)
                    end
                end
            end

            ##
            # :method: each_dynamic_output_port
            # :call-seq:
            #   each_dynamic_output_port(only_self = false) { |port| }
            #
            # Yields all dynamic output ports that are defined on this task context.
            def each_dynamic_output_port(only_self = false)
                return enum_for(:each_dynamic_output_port, only_self) if !block_given?
                each_dynamic_port do |port|
                    if port.kind_of?(OutputPort)
                        yield(port)
                    end
                end
            end

            ##
            # :method: each_dynamic_port
            # :call-seq:
            #   each_dynamic_port(only_self = false) { |port| }
            #
            # Yields all dynamic ports that are defined on this task context.

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
            # :method: each_input_port
            # :call-seq:
            #   each_input_port { |port| }
            #
            # Yields all static input ports that are defined on this task context.

            ##
            # :method: all_input_ports
            # :call-seq:
            #   all_input_ports -> set_of_ports
            #
            # Returns the set of all static input ports that are defined on this task
            # context

            ##
            # :method: self_input_ports
            # :call-seq:
            #   self_input_ports -> set_of_ports
            #
            # Returns the set of static input ports that are added at this level of
            # the model hierarchy. I.e. ports that are defined on this task
            # context, but not on its parent models.

            enumerate_inherited_map("input_port", "input_ports")

            ##
            # :method: each_output_port
            # :call-seq:
            #   each_output_port { |port| }
            #
            # Yields all static output ports that are defined on this task context.

            ##
            # :method: all_output_ports
            # :call-seq:
            #   all_output_ports -> set_of_ports
            #
            # Returns the set of all static output ports that are defined on this task
            # context

            ##
            # :method: self_output_ports
            # :call-seq:
            #   self_output_ports -> set_of_ports
            #
            # Returns the set of static output ports that are added at this level of
            # the model hierarchy. I.e. ports that are defined on this task
            # context, but not on its parent models.

            enumerate_inherited_map("output_port", "output_ports")

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

            enumerate_inherited_map("attribute", "attributes")

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

            enumerate_inherited_map("property", "properties")

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

            enumerate_inherited_map("operation", "operations")

            # Operations that are added by this task context (i.e. operations that are
            # defined there but are not present in the superclass)
            def new_operations
                super_names = superclass.all_operations.map(&:name).to_set
                @operations.values.find_all do |t|
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
	    def output_port(name, type, options = Hash.new)
                name = OroGen.verify_valid_identifier(name)
                check_uniqueness(name)
                options = Kernel.validate_options options,
                    :class => OutputPort

                @output_ports[name] = port = options[:class].new(self, name, type)
                Spec.load_documentation(port, /output_port/)
                port
	    end

	    # call-seq:
	    #	input_port 'name', '/type'
	    #
            # Add a new write port with the given name and type, and returns the
            # corresponding InputPort object.
	    #
	    # See also #output_port
	    def input_port(name, type, options = Hash.new)
                name = OroGen.verify_valid_identifier(name)
                check_uniqueness(name)
                options = Kernel.validate_options options,
                    :class => InputPort

                @input_ports[name] = port = options[:class].new(self, name, type)
                Spec.load_documentation(port, /input_port/)
                port
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
                base_type = project.resolve_type(name)
                full_name = "/boost/shared_ptr<#{base_type.name}>"
                begin
                    project.resolve_type(full_name)
                rescue Typelib::NotFound
                    # HACK: this is needed for the codegen part. Will have to go
                    # HACK: away once we migrate the codegen part to the
                    # HACK: loading infrastructure
                    if project.typekit(true).respond_to?(:shared_ptr)
                        project.typekit(true).shared_ptr(name)
                        project.resolve_type(full_name)
                    else raise
                    end
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
                base_type = project.resolve_type(name)

                full_name = "/RTT/extras/ReadOnlyPointer<#{base_type.name}>"
                begin
                    project.resolve_type(full_name)
                rescue Typelib::NotFound
                    # HACK: this is needed for the codegen part. Will have to go
                    # HACK: away once we migrate the codegen part to the
                    # HACK: loading infrastructure
                    if project.typekit(true).respond_to?(:ro_ptr)
                        project.typekit(true).ro_ptr(name)
                        project.resolve_type(full_name)
                    else raise
                    end
                end
            end

            def all_ports
                all_input_ports + all_output_ports
            end

            def self_ports
                self_input_ports + self_output_ports
            end

            # Enumerates both the input and output ports
            def each_port(&block)
                if !block_given?
                    return enum_for(:each_port, &block)
                end

                each_input_port(&block)
                each_output_port(&block)
            end

            # Finds a port with the given name, and optionally type
            #
            # Returns nil if there are none
            #
            # See also #find_input_port and #find_output_port
            def find_port(name, type = nil)
                p = find_input_port(name) || find_output_port(name)
                if !type || (p && p.type == type)
                    return p
                end
            end

            # Returns true if this task interface has a port named 'name'. If a
            # type is given, the corresponding port will be matched against that
            # type as well
            def has_port?(name, type = nil)
                !!find_port(name, type)
            end

            # Returns true if this task interface has a port named 'name'. If a
            # type is given, the corresponding port will be matched against that
            # type as well
            def has_input_port?(name)
                !!find_input_port(name)
            end

            # Returns true if this task interface has a port named 'name'. If a
            # type is given, the corresponding port will be matched against that
            # type as well
            def has_output_port?(name)
                !!find_output_port(name)
            end


            # Return true if this task interface has an dynamic property.
            def has_dynamic_properties?
                self_properties.each do |p|
                    return true if p.dynamic?
                end
                return false
            end
            
            # Return true if this task interface has an dynamic property.
            def has_dynamic_attributes?
                self_attributes.each do |p|
                    return true if p.dynamic?
                end
                return false
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
            def has_dynamic_port?(name, type)
                has_dynamic_input_port?(name, type) || has_dynamic_output_port?(name, type)
            end

            # Returns the set of dynamic input port definitions that match the
            # given name and type pair. If +type+ is nil, the type is ignored in
            # the matching.
            def find_dynamic_input_ports(name, type)
                if type
                    type = project.find_type(type)
                end
                each_dynamic_input_port.find_all { |p| (!type || !p.type || p.type == type) && p.name === name }
            end

            # Returns true if there is an input port definition that match the
            # given name and type pair. If +type+ is nil, the type is ignored in
            # the matching.
            def has_dynamic_input_port?(name, type = nil)
                !find_dynamic_input_ports(name, type).empty?
            end

            # Returns the set of dynamic output port definitions that match the
            # given name and type pair. If +type+ is nil, the type is ignored in
            # the matching.
            def find_dynamic_output_ports(name, type)
                if type
                    type = project.find_type(type)
                end
                each_dynamic_output_port.find_all { |p| (!type || !p.type || p.type == type) && p.name === name }
            end

            # Returns true if an output port of the given name and type could be
            # created at runtime.
            def has_dynamic_output_port?(name, type = nil)
                !find_dynamic_output_ports(name, type).empty?
            end

            # A set of ports that will trigger this task when they get updated.
            enumerate_inherited_map 'event_port', 'event_ports'

            # Declares that this task context is designed to be woken up when
            # new data is available on one of the given ports (or all already
            # defined ports if no names are given).
            def port_driven(*names)
                default_activity 'triggered'
                names = names.map { |n| n.to_s }
                relevant_ports =
                    if names.empty? then all_input_ports
                    else
                        names.map do |n|
                            obj = find_input_port(n)
                            if !obj
                                if has_output_port?(n)
                                    raise ArgumentError, "#{n} is an output port of #{self.name}, only input ports can be used in #port_driven"
                                else
                                    raise ArgumentError, "#{n} is not a port of #{self.name}"
                                end
                            end
                            obj
                        end
                    end

                relevant_ports.each do |port|
                    port.trigger_port = true
                    @event_ports[port.name] = port
                end
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
            # To configure the activity, you will need to declare the FDs you
            # want to watch in the configureHook():
            #
            #   RTT::extras::FileDescriptorActivity* fd_activity =
            #       getActivity<RTT::extras::FileDescriptorActivity>();
            #   if (fd_activity)
            #       fd_activity->watch(my_fd);
            #
            # Don't forget to remove all watches in cleanupHook with
            #
            #   RTT::extras::FileDescriptorActivity* fd_activity =
            #       getActivity<RTT::extras::FileDescriptorActivity>();
            #   if (fd_activity)
            #       fd_activity->clearAllWatches();
            #
            def fd_driven
                default_activity "fd_driven"
                needs_configuration
            end

            # True if this task context's default activity is a FD-driven activity
            def fd_driven?
                default_activity.first == :fd_driven
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

            # Converts this model into a representation that can be fed to e.g.
            # a JSON dump, that is a hash with pure ruby key / values.
            #
            # The generated hash has the following keys:
            #
            #     name: the name
            #     superclass: the name of this model's superclass (if there is
            #       one)
            #     states: the list of defined states, as formatted by
            #       {each_state}
            #     ports: the list of ports, as formatted by {Port#to_h}
            #     properties: the list of properties, as formatted by
            #       {ConfigurationObject#to_h}
            #     attributes: the list of attributes, as formatted by
            #       {ConfigurationObject#to_h}
            #     operations: the list of operations, as formatted by
            #       {Operation#to_h}
            #
            # @return [Hash]
            def to_h
                Hash[
                    name: name,
                    superclass: superclass.name,
                    states: each_state.to_a,
                    ports: each_port.map(&:to_h),
                    properties: each_property.map(&:to_h),
                    attributes: each_attribute.map(&:to_h),
                    operations: each_operation.map(&:to_h)
                ]
            end
	end
    end
end

