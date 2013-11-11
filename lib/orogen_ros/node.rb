module Orocos::ROS
    module Spec
        OROGEN_ROS_NODE_NAME = "ROS::Node"

        def self.orogen_ros_node
            Orocos::Generation::Project.standard_tasks.find { |t| t.name == OROGEN_ROS_NODE_NAME }
        end

        # Representation of a ROS node as an oroGen model
        class Node < Orocos::Spec::TaskContext

            # @return [String] the name on the ROS side
            attr_accessor :ros_name

            def ros_package
                if @ros_package then @ros_package
                elsif name then @ros_package = Orocos::ROS.rosnode_findpackage(name)
                end
            end

            # @return [String] the actual name of the package this node is
            #   part of
            attr_writer :ros_package

            def initialize(project = nil, name = nil)
                super

                subclasses(Orocos::ROS::Spec.orogen_ros_node)

                @ros_name = name
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
                data_type = ::Orocos::ROS.map_message_type_to_orogen(message_type)
                topic = output_port(name, data_type, :class => OutputTopic)
                topic.ros_name = ::Orocos::ROS.normalize_topic_name(topic_name)
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
                topic.ros_name = ::Orocos::ROS.normalize_topic_name(topic_name)
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

            # Test if there is a specification available for this node
            #
            # @return [Boolean] True if this object has been generated from spec, False
            #   otherwise
            def spec_available?
                if !@spec_available and ros_name
                    @spec_available = Orocos::ROS.node_spec_available?(ros_name)
                end
                @spec_available
            end
        end
    end
end
