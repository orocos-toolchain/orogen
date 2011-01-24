module Orocos
    module Spec
        # Specification for an output port
        class OutputPort < Port
            def initialize(*args)
                super
                @sample_size  = 1
                @period = 1
                @burst_size   = 0
                @burst_period = 0
                @port_triggers = Set.new
                @triggered_on_update = nil
                @max_sizes = Hash.new
            end

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
                if @triggered_once_per_update
                    return []
                end

                @port_triggers
            end

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
                    if values.size == 1
                        values[0].to_hash
                    elsif values.size == 2
                        values[1].to_hash.merge('' => values[0])
                    end

                values.each do |name, value|
                    resolve_max_size_path(resolved_type, name)
                    value.to_int
                end
                puts values.inspect
                values
            end

            def self.initialize_max_size_sample(path, sample, max_sizes)
                sample_t = sample.class
                if !(sample_t <= Typelib::Type) || !sample_t.contains?(Typelib::ContainerType)
                    return true
                end

                path = path.dup
                puts "#{path.join(".")} #{sample}"

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
                input_ports = input_ports.to_set.map do |name|
                    if !(p = task.find_input_port(name))
                        raise ArgumentError, "#{name} is not an input port of #{self}"
                    end
                    p
                end

                @port_triggers |= input_ports
                self
            end

            # Used to write the triggered_on_update flag directly. This should
            # not be used in normal situations
            attr_writer :triggered_on_update

            # Declares that this port will be written for each call of the
            # updateHook(). It is the default if #triggered_on has not been
            # called.
            def triggered_on_update
                @triggered_on_update = true
                self
            end

            # Declares that at most one sample will be written per call to
            # updateHook, regardless of the actual amount of samples that are
            # waiting to be read by the task
            def triggered_once_per_update
                @triggered_once_per_update = true
                self
            end

            # True if the port will be written for the calls to updateHook()
            # that are triggered by the activity.
            #
            # See #triggered_on_update and #triggered_on
            def triggered_on_update?
                if @triggered_once_per_update then true
                elsif port_triggers.empty?
                    # One can set triggered_on_update to false explicitely to
                    # override the default
                    @triggered_on_update != false
                else
                    @triggered_on_update
                end
            end

            # If true, this port will be written at most once per call to
            # updateHook, regardless of the actual amount of samples that are
            # waiting to be read by the task
            #
            # The port period and burst are still used
            def triggered_once_per_update?
                !!@triggered_once_per_update
            end
        end
    end
end


