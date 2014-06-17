# NOTE: the setup of TypekitMarshallers (e.g. for logging and backward
# compatibility between the Orocos and OroGen toplevel namespaces) is done in
# orogen.rb. This is required as we want to be able to load single marshallers
# files
require 'orogen/marshallers/corba'
require 'orogen/marshallers/type_info'
require 'orogen/marshallers/mqueue'
require 'orogen/marshallers/ros'
require 'orogen/marshallers/typelib'
require 'orogen/marshallers/iostream'

