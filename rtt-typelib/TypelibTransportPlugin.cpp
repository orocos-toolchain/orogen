#include "TypelibTransportPlugin.hpp"
#include <typelib/pluginmanager.hh>
#include <rtt/Logger.hpp>
#include <typelib/registry.hh>

#ifdef HAS_ROSLIB
#include <ros/package.h>
#endif

using namespace orogen_transports;
using namespace RTT;

TypelibTransportPlugin::TypelibTransportPlugin(std::string const& basename)
    : m_basename(basename), m_registry(0) {}
TypelibTransportPlugin::~TypelibTransportPlugin()
{
    delete m_registry;
}

bool TypelibTransportPlugin::loadRegistry()
{
    std::string path = getTlbPath();
    try
    {
        m_registry = Typelib::PluginManager::load("tlb", path);
        return true;
    }
    catch(std::exception const& e) {
        log(Error) << "cannot load the typekit's Typelib registry from" << endlog();
        log(Error) << "  " << path << endlog();
#ifndef HAS_ROSLIB
        log(Error) << "remember to do 'make install' before you use the oroGen-generated libraries ?" << endlog();
#endif
        log(Error) << endlog();
        log(Error) << "the Typelib transport will not be available for types defined in this typekit" << endlog();
    }
    return false;
}

std::string orogen_transports::TypelibTransportPlugin::getTransportName() const
{ return "Typelib"; }
std::string orogen_transports::TypelibTransportPlugin::getTypekitName() const
{ return "/orogen/" + m_basename; }
std::string orogen_transports::TypelibTransportPlugin::getName() const
{ return "/orogen/" + m_basename + "/TYPELIB"; }

