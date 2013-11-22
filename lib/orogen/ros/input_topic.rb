module OroGen
    module ROS
        class InputTopic < Orocos::Spec::InputPort
            # @return [String] the actual name on the ROS side
            #   {name} returns the node-local name (the "port" name)
            attr_accessor :ros_name

            alias :topic_name :ros_name
        end
    end
end

