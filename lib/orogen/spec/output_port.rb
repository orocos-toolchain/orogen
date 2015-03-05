module OroGen
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


