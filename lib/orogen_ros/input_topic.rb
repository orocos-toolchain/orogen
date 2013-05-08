module Orocos::ROS
    module Spec
        class InputTopic < Orocos::Spec::InputPort
            # @return [String] the actual name on the ROS side
            #   {name} returns the node-local name (the "port" name)
            attr_accessor :ros_name
        end
    end
end

