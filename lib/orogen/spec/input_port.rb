module OroGen
    module Spec
        # Specification for an input port
        class InputPort < Port
            attr_reader :required_connection_type

            def initialize(*args)
                super
                @required_connection_type = :data
            end

            attr_predicate :trigger_port?, true

            # If called, this port will be registered on the task as a trigger
            # port,
            #
            # i.e. the following
            #
            #   task_context "Name" do
            #     input_port("port", "int").
            #       trigger_port
            #   end
            #
            # is equivalent to
            #
            #   task_context "Name" do
            #     input_port("port", "int")
            #     port_driven 'port'
            #   end
            #
            # The difference with port_driven is that it works on dynamic ports
            # as well
            def task_trigger
                task.port_driven(name)
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

            # Returns true if the task context requires connections to this port to
            # be reliable (i.e. non-lossy).
            #
            # See #needs_reliable_policy for more information
            def needs_reliable_connection?; @needs_reliable_connection end

            # Declares that the task context requires a non-lossy policy
            #
            # This is different from #requires_buffered_connection as a data
            # policy could be used if the period of the connection's source is
            # much longer than the period of the connection's end (for
            # instance).
            def needs_reliable_connection; @needs_reliable_connection = true; self end

            # In oroGen, input ports are cleared in the startHook()
            #
            # Calling #do_not_clean disables this behaviour for this particular
            # port
            def do_not_clean
                @do_not_clean = true
            end

            # If true, this port accepts to have multiple active connections at the same time
            def multiplexes?
                !!@multiplexes
            end

            # Declares that this port accepts multiple active connections
            def multiplexes
                @multiplexes = true
                self
            end
        end
    end
end

