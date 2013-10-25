require 'orogen'
require 'orogen_ros/base'

# Add standard task specification for ros
Orocos::Generation::Project.add_standard_task_spec("ros.orogen",Orocos::ROS::OROGEN_ROS_LIB_DIR)

require 'orogen_ros/package'
require 'orogen_ros/project'
require 'orogen_ros/input_topic'
require 'orogen_ros/output_topic'
require 'orogen_ros/node'
require 'orogen_ros/launch'

# The toplevel namespace for orogen_ros
#
# You should describe the basic idea about orogen_ros here
require 'utilrb/logger'
module Orocos::ROS
    extend Logger::Root('Orocos::ROS', Logger::WARN)
end

