module Orocos
    module Spec
        # Generic representation of ports. The actual ports are either
        # instance of InputPort or OutputPort
	class Port
	    # The port task
	    attr_reader :task
	    # The port name
	    attr_reader :name
	    # The port type. It can be nil for dynamic ports
	    attr_reader :type
            # The port type name
            def type_name; type.name end

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

            # True if this is a dynamic port model, false otherwise
            def dynamic?; false end

	    def initialize(task, name, type)
                if !name.kind_of?(Regexp)
                    name = name.to_s
                    if name !~ /^\w+$/
                        raise ArgumentError, "port names need to be valid C++ identifiers, i.e. contain only alphanumeric characters and _ (got #{name})"
                    end
                elsif !type && !dynamic?
                    raise ArgumentError, "only dynamic ports are allowed to have no type"
                end

                if type
                    type = task.component.find_interface_type(type)
                    Orocos.validate_toplevel_type(type)
                    if type.name == "/std/vector<double>"
                        Orocos::Generation.warn "#{type.name} is used as the port type for #{name}, logging it will not be possible"
                    end
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
    end
end


