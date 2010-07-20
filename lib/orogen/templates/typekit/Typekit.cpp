// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include "<%= component.name %>Typekit.hpp"

#include <iostream>
#include <boost/cstdint.hpp>
#include <boost/lexical_cast.hpp>

#include <rtt/types/TypeInfoRepository.hpp>
#include "typekit/type_info/<%= component.name %>TypeInfo.hpp"

using namespace RTT;

orogen_typekits::<%= component.name %>TypekitPlugin::<%= component.name %>TypekitPlugin()
{}

orogen_typekits::<%= component.name %>TypekitPlugin::~<%= component.name %>TypekitPlugin()
{}


#define TOOLKIT_PACKAGE_NAME_aux0(target) #target
#define TOOLKIT_PACKAGE_NAME_aux(target) "<%= component.name %>-typekit-" TOOLKIT_PACKAGE_NAME_aux0(target)
#define TOOLKIT_PACKAGE_NAME TOOLKIT_PACKAGE_NAME_aux(OROCOS_TARGET)
bool orogen_typekits::<%= component.name %>TypekitPlugin::loadTypes()
{
    RTT::types::TypeInfoRepository::shared_ptr ti_repository = RTT::types::TypeInfoRepository::Instance();

    RTT::types::TypeInfo* ti = 0;
    <% registered_types.each do |type| %>
    ti = <%= type.method_name(true) %>_TypeInfo();
    ti_repository->addType( ti );
    <% end %>
    <% array_types.each do |type| %>
    ti = <%= type.deference.method_name(true) %>_ArrayTypeInfo();
    ti_repository->addType( ti );
    <% end %>

    return true;
}

bool orogen_typekits::<%= component.name %>TypekitPlugin::loadOperators()
{ return true; }
bool orogen_typekits::<%= component.name %>TypekitPlugin::loadConstructors()
{ return true; }
std::string orogen_typekits::<%= component.name %>TypekitPlugin::getName()
{ return "/orogen/<%= component.name %>"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::<%= component.name %>TypekitPlugin);

