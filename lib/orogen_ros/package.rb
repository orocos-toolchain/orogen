# This is going to be the main namespace for your project
module Orocos::ROS
    module Spec
        # Used to extend the Project class when loading an oroGen project that
        # represents a ROS package
        module Package
            # Create a model for a ROS node
            #
            # @param [String] name the name of the node
            # @return [ROSNode]
            def ros_node(name, &block)
                external_task_context(name, :class => Node, &block)
            end

        end
    end
end

