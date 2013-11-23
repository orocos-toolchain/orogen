module OroGen
    module ROS
    module Spec
        # Representation of a ROS node as an oroGen model
        class Node < OroGen::Spec::TaskContext
            # @return [String] the name on the ROS side
            attr_accessor :ros_name
            # @return [String] the name of the ROS package
            attr_accessor :ros_package

            def initialize(project, name = nil)
                super

                @ros_name = name
                @ros_package = project.name
            end

            # Declares that this node produces data on a specific topic
            #
            # @param [String] topic_name the topic name
            # @param [String] name the name on which this topic should be
            #   registered on self
            # @param [String] message_type the name of the ROS message type. A
            #   ROS mapping must exist for it
            # @return [OutputTopic]
            def output_topic(topic_name, name, message_type)
                data_type = project.ros_loader.map_message_type_to_orogen(message_type)
                topic = output_port(name, data_type, :class => OutputTopic)
                topic.ros_name = ROS.normalize_topic_name(topic_name)
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
                data_type = project.ros_loader.map_message_type_to_orogen(message_type)
                topic = input_port(name, data_type, :class => InputTopic)
                topic.ros_name = ROS.normalize_topic_name(topic_name)
                topic
            end

            def to_s
                "#{self.class} ros_name: #{ros_name}, ros_package: #{ros_package}"
            end

            # Equals operator
            # @return [Boolean] True, if ros node and package name equal equal, False otherwise
            def ==(other)
                if other.respond_to?(:ros_name)
                    if ros_name.empty? || other.ros_name.empty?
                        raise ArgumentError, "Cannot compare ROS Spec node with empty ros_name: #{self} vs. #{other}"
                    end
                    return ros_name == other.ros_name && ros_package == other.ros_package
                end
                false
            end

            def eql?(other); self == other end
        end
    end
    end
end
