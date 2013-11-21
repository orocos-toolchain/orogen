module OroGen
    # Integration of ways to load oroGen models
    module Loaders
        extend Logger::Hierarchy
    end
end

require 'orogen/loaders/base'
require 'orogen/loaders/pkg_config'
require 'orogen/loaders/files'
require 'orogen/loaders/project'
require 'orogen/loaders/aggregate'
require 'orogen/loaders/rtt'
