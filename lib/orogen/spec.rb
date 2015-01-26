require 'utilrb/logger'
module OroGen
    module Spec
        extend Logger::Hierarchy
    end
end

require 'orogen/spec/configuration_object'
require 'orogen/spec/property'
require 'orogen/spec/attribute'
require 'orogen/spec/opaque_definition'

require 'orogen/spec/operation'

require 'orogen/spec/port'
require 'orogen/spec/input_port'
require 'orogen/spec/output_port'
require 'orogen/spec/dynamic_ports'

require 'orogen/spec/task_context'
require 'orogen/spec/deployment'

require 'orogen/spec/typekit'
require 'orogen/spec/project'

require 'orogen/spec/doc'
