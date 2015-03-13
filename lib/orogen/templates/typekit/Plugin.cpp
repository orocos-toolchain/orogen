/* Generated from orogen/lib/orogen/templates/typekit/Plugin.cpp */

// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include <<%= typekit.name %>/typekit/Plugin.hpp>

#include <boost/lexical_cast.hpp>

#include <rtt/types/TypeInfoRepository.hpp>
#include "type_info/Registration.hpp"

using namespace RTT;

orogen_typekits::<%= typekit.name %>TypekitPlugin::<%= typekit.name %>TypekitPlugin()
{}

orogen_typekits::<%= typekit.name %>TypekitPlugin::~<%= typekit.name %>TypekitPlugin()
{}


#define TYPEKIT_PACKAGE_NAME_aux0(target) #target
#define TYPEKIT_PACKAGE_NAME_aux(target) "<%= typekit.name %>-typekit-" TYPEKIT_PACKAGE_NAME_aux0(target)
#define TYPEKIT_PACKAGE_NAME TYPEKIT_PACKAGE_NAME_aux(OROCOS_TARGET)
bool orogen_typekits::<%= typekit.name %>TypekitPlugin::loadTypes()
{
    RTT::types::TypeInfoRepository::shared_ptr ti_repository = RTT::types::TypeInfoRepository::Instance();

    RTT::types::TypeInfoGenerator* ti = 0;
    <% registered_types.each do |type| %>
        <% if type < Typelib::ArrayType %>
    ti = <%= type.deference.method_name(true) %>_ArrayTypeInfo();
    ti_repository->addType( ti );
        <% else %>
    ti = <%= type.method_name(true) %>_TypeInfo();
    ti_repository->addType( ti );
        <% end %>
    <% end %>

    return true;
}

bool orogen_typekits::<%= typekit.name %>TypekitPlugin::loadOperators()
{ return true; }
bool orogen_typekits::<%= typekit.name %>TypekitPlugin::loadConstructors()
{ return true; }
std::string orogen_typekits::<%= typekit.name %>TypekitPlugin::getName()
{ return "<%= typekit.name %>"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::<%= typekit.name %>TypekitPlugin);

