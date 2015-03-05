require 'orogen'

module OroGen
    module Gen
        extend Logger::Hierarchy

        module RTT_CPP
            extend Logger::Hierarchy

            ConfigError = OroGen::ConfigError
            OROGEN_LIB_DIR = OroGen::OROGEN_LIB_DIR

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
OroGen::Gen::RTT_CPP::Typekit.register_plugin(OroGen::TypekitMarshallers::ROS::Plugin)
OroGen::Gen::RTT_CPP::Typekit.register_plugin(OroGen::TypekitMarshallers::Corba::Plugin)
OroGen::Gen::RTT_CPP::Typekit.register_plugin(OroGen::TypekitMarshallers::MQueue::Plugin)
OroGen::Gen::RTT_CPP::Typekit.register_plugin(OroGen::TypekitMarshallers::TypeInfo::Plugin)
OroGen::Gen::RTT_CPP::Typekit.register_plugin(OroGen::TypekitMarshallers::TypelibMarshaller::Plugin)
