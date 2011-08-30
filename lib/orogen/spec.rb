require 'utilrb/logger'
module Orocos
    module Spec
	class << self
	    attr_reader :logger
	end
	@logger = Logger.new(STDOUT)
	logger.level = Logger::WARN
        logger.formatter = lambda { |severity, time, progname, msg| "#{severity}: #{msg}\n" }
	extend Logger::Forward
    end
end

require 'orogen/spec/configuration_object'
require 'orogen/spec/property'
require 'orogen/spec/attribute'

require 'orogen/spec/operation'

require 'orogen/spec/port'
require 'orogen/spec/input_port'
require 'orogen/spec/output_port'
require 'orogen/spec/dynamic_ports'

require 'orogen/spec/task_context'
