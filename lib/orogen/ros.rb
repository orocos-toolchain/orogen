module OroGen
    # Toplevel module for all the functionality that allows to describe ROS
    # nodes using oroGen models
    module ROS
        extend Logger::Hierarchy
    end
end

require 'orogen/marshallers/ros'
require 'orogen/ros/base'
require 'orogen/ros/loader'
require 'orogen/ros/default_loader'
require 'orogen/ros/spec'

