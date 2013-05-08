module Orocos
    module Spec
        # Representation of a ROS node as an oroGen model
        class ROSNode < TaskContext
            # @return [String] the actual name on the ROS side
            #   {name} returns the node-local name (the "port" name)
            attr_accessor :ros_name

            # Declares that this node produces data on a specific topic
            #
            # @param [String] topic_name the topic name
            # @param [String] name the name on which this topic should be
            #   registered on self
            # @param [String] message_type the name of the ROS message type. A
            #   ROS mapping must exist for it
            # @return [OutputTopic]
            def output_topic(topic_name, name, message_type)
                data_type = project.map_message_type_to_orogen(message_type)
                topic = super(name, data_type, :class => OutputTopic)
                topic.ros_name = topic_name
                topic
            end

            # Declares that this node subscribes to a specific topic
            #
            # @param [String] topic_name the topic name
            # @param [String] name the name on which this topic should be
            #   registered on self
            # @param [String] message_type the name of the ROS message type. A
            #   ROS mapping must exist for it
            # @return [InputTopic]
            def input_topic(topic_name, name, message_type)
                data_type = project.map_message_type_to_orogen(message_type)
                topic = super(name, data_type, :class => InputTopic)
                topic.ros_name = topic_name
                topic
            end
        end
    end
end
