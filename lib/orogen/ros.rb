module OroGen
    # Toplevel module for all the functionality that allows to describe ROS
    # nodes using oroGen models
    module ROS
        extend Logger::Hierarchy
    end
end

require 'orogen/ros/base'
require 'orogen/ros/package'
require 'orogen/ros/input_topic'
require 'orogen/ros/output_topic'
require 'orogen/ros/node'
require 'orogen/ros/launcher'
require 'orogen/ros/loader'

