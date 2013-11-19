require 'orogen'

module OroGen
    module Gen
        extend Logger::Hierarchy

        module RTT_CPP
            extend Logger::Hierarchy

            ConfigurationObject = Spec::ConfigurationObject
            Attribute           = Spec::Attribute
            Property            = Spec::Property

            Operation           = Spec::Operation

            Port                = Spec::Port
            OutputPort          = Spec::OutputPort
            InputPort           = Spec::InputPort
            DynamicInputPort    = Spec::DynamicInputPort
            DynamicOutputPort   = Spec::DynamicInputPort

            TaskContext         = Spec::TaskContext
        end
    end
    Generation = Gen::RTT_CPP
end

require 'orogen/gen/enable'
require 'orogen/gen/base'
require 'orogen/gen/templates'
require 'orogen/gen/typekit'
require 'orogen/marshallers'
require 'orogen/gen/deployment'
require 'orogen/gen/tasks'
require 'orogen/gen/project'
require 'orogen/gen/imports'
