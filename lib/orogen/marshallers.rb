module OroGen
    # Code-generation support for the different supported RTT transports
    module TypekitMarshallers
        extend Logger::Hierarchy
    end
end

Orocos::TypekitMarshallers = OroGen::TypekitMarshallers

require 'orogen/marshallers/corba'
require 'orogen/marshallers/type_info'
require 'orogen/marshallers/mqueue'
require 'orogen/marshallers/ros'
require 'orogen/marshallers/typelib'
require 'orogen/marshallers/iostream'

