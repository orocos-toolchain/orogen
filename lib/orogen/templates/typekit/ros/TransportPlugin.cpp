/* Generated from orogen/lib/orogen/templates/typekit/ros/TransportPlugin.cpp */

// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include "transports/ros/Registration.hpp"
#include "transports/ros/TransportPlugin.hpp"
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/transports/ros/RosLib.hpp>
using namespace RTT;

bool orogen_typekits::<%= typekit.name %>RosTransportPlugin::registerTransport(std::string type_name, RTT::types::TypeInfo* ti)
{
    if(ti->hasProtocol(ORO_ROS_PROTOCOL_ID))
	return false;

    <% first_type = true;
       typesets.interface_types.each do |type|
           next if !ros_exported_type?(type)
           names = [type.name]
           if aliases = typesets.aliases[type]
               names.concat(aliases)
           end
           if_cond = names.map { |n| "\"#{n}\" == type_name" }.join(" || ")
        %>
    <%= 'else ' unless first_type %>if (<%= if_cond %>)
    {
        return ti->addProtocol(ORO_ROS_PROTOCOL_ID,
            <%= type.method_name %>_ROSTransport());
    }
    <% first_type = false
    end %>
    return false;
}
std::string orogen_typekits::<%= typekit.name %>RosTransportPlugin::getTransportName() const
{ return "ROS"; }
std::string orogen_typekits::<%= typekit.name %>RosTransportPlugin::getTypekitName() const
{ return "/orogen/<%= typekit.name %>"; }
std::string orogen_typekits::<%= typekit.name %>RosTransportPlugin::getName() const
{ return "/orogen/<%= typekit.name %>/ROS"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::<%= typekit.name %>RosTransportPlugin);

