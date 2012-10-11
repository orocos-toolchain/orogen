/* Generated from orogen/lib/orogen/templates/typekit/typelib/TransportPlugin.cpp */

#include "Registration.hpp"
#include "TransportPlugin.hpp"
#include <rtt/typelib/TypelibMarshallerBase.hpp>
#include <rtt/Logger.hpp>
#include <rtt/types/TypekitPlugin.hpp>
using namespace RTT;
#ifdef HAS_ROSLIB
#include <ros/package.h>
#endif

#define TYPEKIT_PACKAGE_NAME_aux0(target) #target
#define TYPEKIT_PACKAGE_NAME_aux(target) "<%= typekit.name %>-typekit-" TYPEKIT_PACKAGE_NAME_aux0(target)
#define TYPEKIT_PACKAGE_NAME TYPEKIT_PACKAGE_NAME_aux(OROCOS_TARGET)

std::string orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::getTypelibRegistryPath()
{
#ifdef HAS_ROSLIB
    /** If ROSLIB is available, we'll use ros::package to find the path to a package with the
      same name as our typekit. This may only be enabled if IS_ROS_PACKAGE is true at build time ! **/
    using namespace ros::package;
    try {
        bool all_good = true, found = false;
        std::string ppath = getPath( "<%= typekit.name %>" );
        if ( !ppath.empty() ) {
            return ppath + "/typekit" + "/<%= typekit.name %>.tlb";
        } else
            log(Debug) << "Not a ros package: " << "<%= typekit.name %>" << endlog();
    } catch(...) {
        log(Debug) << "Not a ros package: " << "<%= typekit.name %>" << endlog();
    }
#endif
    return TYPEKIT_REGISTRY;
}

std::string orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::getTlbPath() const
{
    return <%= typekit.name %>TypelibTransportPlugin::getTypelibRegistryPath();
}

orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::<%= typekit.name %>TypelibTransportPlugin()
    : TypelibTransportPlugin("<%= typekit.name %>") {}

bool orogen_typekits::<%= typekit.name %>TypelibTransportPlugin::registerTransport(std::string type_name, RTT::types::TypeInfo* ti)
{
    if (!m_registry)
    {
        if (!loadRegistry())
            return false;
    }
    
    if(ti->hasProtocol(orogen_transports::TYPELIB_MARSHALLER_ID))
	return false;

    <% first_type = true;
       typesets.interface_types.each do |type|
           names = [type.name]
           if aliases = typesets.aliases[type]
               names.concat(aliases)
           end
           if_cond = names.map { |n| "\"#{n}\" == type_name" }.join(" || ")
        %>
    <%= 'else ' unless first_type %>if (<%= if_cond %>)
    {
        try {
            ti->addProtocol(orogen_transports::TYPELIB_MARSHALLER_ID,
                <%= type.method_name %>_TypelibMarshaller(*m_registry));
        }
        catch(std::runtime_error const& e)
        {
            log(Error) << "cannot register a typelib transport for " << type_name << endlog();
            log(Error) << "  the following error occured:" << endlog();
            log(Error) << "  " << e.what() << endlog();
            log(Error) << "  the registry can be found at " << TYPEKIT_REGISTRY << endlog();
            return false;
        }
        return true;
    }
    <% first_type = false
    end %>
    return false;
}

ORO_TYPEKIT_PLUGIN(orogen_typekits::<%= typekit.name %>TypelibTransportPlugin);

