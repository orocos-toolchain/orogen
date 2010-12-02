module Orocos
    module Spec
        # Specification for an input port
        class InputPort < Port
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

            def register_for_generation
                super
                task.in_base_hook('start', "_#{name}.clear();")
            end
        end
    end
end

