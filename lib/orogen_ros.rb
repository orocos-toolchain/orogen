require 'orogen_ros/package'
require 'orogen_ros/input_topic'
require 'orogen_ros/output_topic'
require 'orogen_ros/node'

# The toplevel namespace for orogen_ros
#
# You should describe the basic idea about orogen_ros here
require 'utilrb/logger'
module Orogen::ROS
    extend Logger::Root('Orogen::ROS', Logger::WARN)
end

