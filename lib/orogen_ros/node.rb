module Orocos::ROS
    module Spec
        # Representation of a ROS node as an oroGen model
        class Node < Orocos::Spec::TaskContext
            # @return [String] the name on the ROS side
            attr_accessor :ros_name

            # @return [String] the actual name of the package this node is
            #   part of
            attr_accessor :ros_package

            # Declares that this node produces data on a specific topic
            #
            # @param [String] topic_name the topic name
            # @param [String] name the name on which this topic should be
            #   registered on self
            # @param [String] message_type the name of the ROS message type. A
            #   ROS mapping must exist for it
            # @return [OutputTopic]
            def output_topic(topic_name, name, message_type)
                data_type = ::Orocos::ROS.map_message_type_to_orogen(message_type)
                topic = output_port(name, data_type, :class => OutputTopic)
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
                data_type = ::Orocos::ROS.map_message_type_to_orogen(message_type)
                topic = input_port(name, data_type, :class => InputTopic)
                topic.ros_name = topic_name
                topic
            end

            def to_s
                "#{self.class} ros_name: #{ros_name}, ros_package: #{ros_package}"
            end
        end
    end
end
