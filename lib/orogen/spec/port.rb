module OroGen
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
            # The port name as it is registered on RTT
            def orocos_type_name
                Typelib::Registry.rtt_typename(type)
            end

            # Converts this model into a representation that can be fed to e.g.
            # a JSON dump, that is a hash with pure ruby key / values.
            #
            # The generated hash has the following keys:
            #
            #     name: the name
            #     type: the type (as marshalled with Typelib::Type#to_h)
            #     direction: either the string 'input' or 'output'
            #     doc: the documentation string
            #
            # @return [Hash]
            def to_h
                Hash[
                    direction: (if kind_of?(OutputPort) then 'output' else 'input' end),
                    name: name,
                    type: type.to_h,
                    doc: (doc || "")
                ]
            end


            # Stores the policy for keeping last values. It can be nil, :initial or true
            #
            # The default is :initial. It reasonably ensures that connections
            # containing variable-sized vectors such as std::vector are properly
            # initialized, avoiding memory allocations. Turn it off for ports
            # on which huge data samples are going to be sent and/or ports for
            # which realtime communication is not required
            dsl_attribute :keep_last_written_value do |value|
                case value
                when NilClass, TrueClass, FalseClass
                when :initial
                else
                    raise ArgumentError, "keep_last_written_value can only be one of true, false/nil and :initial. Got #{value}"
                end
                value
            end

            # True if the task context supports only static connections on this
            # port, and false otherwise
            #
            # See #static for more details.
            def static?; !!@static end

            # Declares that this port can be connected/disconnected only when
            # the task context is in a non-running state.
            #
            # The default is that the port is dynamic, i.e. can be
            # connected/disconnected regardless of the task context's state.
            #
            # See also #dynamic
            def static; @static = true end

            # Declares that this port can be connected/disconnected while the
            # task context is running. It is the opposite of #static.
            #
            # This is the default
            def dynamic; @static = false end

            def pretty_print(pp)
                pp.text "[#{self.kind_of?(InputPort) ? "in" : "out"}]#{name}:#{type_name}"
            end

            # True if this is a dynamic port model, false otherwise
            def dynamic?; false end

	    def initialize(task, name, type, options = Hash.new)
                if !name.kind_of?(Regexp)
                    name = name.to_s
                    if name !~ /^\w+$/
                        raise ArgumentError, "port names need to be valid C++ identifiers, i.e. contain only alphanumeric characters and _ (got #{name})"
                    end
                elsif !type && !dynamic?
                    raise ArgumentError, "only dynamic ports are allowed to have no type"
                end

                if type
                    type = task.project.find_interface_type(type)
                    OroGen.validate_toplevel_type(type)
                    if type.name == "/std/vector<double>"
                        Spec.warn "#{type.name} is used as the port type for #{name}, logging it will not be possible"
                    end
                end
		@task, @name, @type = task, name, type

                @doc = nil
                @max_sizes = Hash.new
                keep_last_written_value :initial
	    end

	    # call-seq:
	    #	doc new_doc -> self
            #	doc ->  current_doc
	    #
	    # Gets/sets a string describing this object
	    dsl_attribute(:doc) { |value| value.to_s }

            def self.resolve_max_size_path(type, name)
                resolved_type = name.split('.').inject(type) do |resolved_type, element|
                    match_deference = element.match(/(\[\])*$/)

                    if !match_deference.pre_match.empty?
                        resolved_type = resolved_type[match_deference.pre_match]
                    end

                    (match_deference[0].length / 2).times do
                        resolved_type = resolved_type.deference
                    end
                    resolved_type
                end

                if !(resolved_type <= Typelib::ContainerType)
                    raise ArgumentError, "#{name} resolves to the #{resolved_type.name} type in #{type.name}, which is not a variable size container"
                end
                resolved_type
            end

            def self.validate_max_sizes_spec(resolved_type, values)
                values =
                    if values.respond_to?(:to_ary)
                        # values is either [spec] or [vector_size, spec]
                        # This is used by dsl_attribute implementations of
                        # max_sizes

                        if values.size == 1
                            values[0].to_hash
                        elsif values.size == 2
                            values[1].to_hash.merge('' => values[0])
                        end
                    elsif values.respond_to?(:to_hash)
                        # Direct call
                        values.to_hash
                    else
                        raise ArgumentError, "expected a one or two element array or a hash, got #{values.inspect}"
                    end

                values.each do |name, value|
                    if resolved_type
                        resolve_max_size_path(resolved_type, name)
                    end
                    value.to_int
                end
                values
            end

            def self.initialize_max_size_sample(path, sample, max_sizes)
                sample_t = sample.class
                if !(sample_t <= Typelib::Type) || !sample_t.contains?(Typelib::ContainerType)
                    return true
                end

                path = path.dup

                if sample.kind_of?(Typelib::ContainerType)
                    max_size = max_sizes[path.join(".")]
                    if !max_size
                        return false
                    end

                    if path.empty? then path = ["[]"]
                    else path[-1] = "#{path[-1]}[]"
                    end
                    element = sample_t.deference.new
                    if !initialize_max_size_sample(path, element, max_sizes)
                        return false
                    end

                    max_size.times do
                        sample << element
                    end
                    return true

                elsif sample.kind_of?(Typelib::ArrayType)
                    if path.empty? then path = ["[]"]
                    else path[-1] = "#{path[-1]}[]"
                    end
                    element = sample_t.deference.new
                    if !initialize_max_size_sample(path, element, max_sizes)
                        return false
                    end

                    sample_t.size.times do |i|
                        sample[i] = element
                    end
                    return true

                elsif sample.kind_of?(Typelib::CompoundType)
                    path << nil
                    sample_t.each_field do |field_name, _|
                        field = sample.raw_get_field(field_name)
                        path[-1] = field_name
                        if !initialize_max_size_sample(path, field, max_sizes)
                            return false
                        end
                    end
                    return true
                end

                raise InternalError, "unknown case in initialize_max_size_sample for #{path.join(".")} of type #{sample_t.name}"
            end

            # Returns the maximum size of a marshalled sample coming out of this
            # port
            #
            # If the type on this port contains variable-size containers, this
            # method will return nil unless the 'max_sizes' map specifies their
            # size
            def self.compute_max_marshalling_size(type, max_sizes)
                sample = type.new

                path = []
                if initialize_max_size_sample(path, sample, max_sizes)
                    sample.to_byte_array(:remove_trailing_skips => false).length
                end
            end

            ##
            # :method: max_sizes
            #
            # :call-seq:
            #   max_sizes('name.to[].field' => value, 'name.other' => value) => self
            #   max_sizes(10, 'name.to[].field' => value, 'name.other' => value) => self
            #   max_sizes => current size specification
            #
            # Sets the maximum allowed size for the variable-size containers in
            # +type+. The mapping is given as path.to.field => size, where [] is
            # used to get elements of an array or variable-size container.
            #
            # If the port type is a container itself, the second form is used,
            # where the first argument is the container size and the rest
            # specifies its element sizes (and must start with [])
            #
            # For instance, with the types
            #
            #   struct A
            #   {
            #       std::vector<int> values;
            #   };
            #   struct B
            #   {
            #       std::vector<A> field;
            #   };
            #
            # Then sizes on a port of type B would be given with
            #
            #   port.max_sizes('field' => 10, 'field[].values' => 20)
            #
            # while the sizes on a port of type std::vector<A> would be given
            # with
            #
            #   port.max_sizes(10, '[].values' => 20)
            #
            # Finally, on a port of type std::vector<std::vector<A>>, one would
            # use
            #
            #   port.max_sizes(10, '[]' => 20, '[][].values' => 30)
            #
            dsl_attribute :max_sizes do |*values|
                # Validate that all values are integers and all names map to
                # known types
                resolved_type = task.project.intermediate_type_for(type)
                values = OutputPort.validate_max_sizes_spec(resolved_type, values)
                @max_sizes.merge(values)
            end

            # Returns the maximum marshalled size of a sample from this port, as
            # marshalled by typelib
            #
            # If the type contains variable-size containers, the result is
            # dependent on the values given to #max_sizes. If not enough is
            # known, this method will return nil.
            def max_marshalling_size
                resolved_type = task.project.intermediate_type_for(type)
                OutputPort.compute_max_marshalling_size(resolved_type, max_sizes)
            end
	end
    end
end


