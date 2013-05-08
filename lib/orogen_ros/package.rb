# This is going to be the main namespace for your project
module Orocos
    module Spec
        # Used to extend the Project class when loading an oroGen project that
        # represents a ROS package
        module ROSPackage
            # @return [String] the type name that should be used on the oroGen
            #   side to represent the given ROS message
            #
            # @param [String] message_type the ROS message type name
            def map_ros_message_type_to_orogen(message_type)
                if !@ros_marshaller
                    @ros_marshaller ||= Orocos::TypekitMarshallers::ROS.new(self)
                    @ros_marshaller.load_ros_mappings
                end
                msg_name, _ = type_mappings.find { |type_name, msg_name| msg_name = message_type }
                if !msg_name
                    raise ArgumentError, "there are not oroGen equivalent for #{message_type}"
                end
                msg_name
            end

            # Create a model for a ROS node
            #
            # @param [String] name the name of the node
            # @return [ROSNode]
            def ros_node(name, &block)
                external_task_context(name, :type => ROSNode, &block)
            end
        end
    end
end

