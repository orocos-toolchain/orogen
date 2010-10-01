/* Generated from orogen/lib/orogen/templates/typekit/Plugin.cpp */

// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include "Plugin.hpp"

#include <iostream>
#include <boost/cstdint.hpp>
#include <boost/lexical_cast.hpp>

#include <rtt/types/TypeInfoRepository.hpp>
#include "type_info/Registration.hpp"
#include "Types.hpp"
#include <rtt/types/SequenceConstructor.hpp>
#include <rtt/types/TemplateConstructor.hpp>

using namespace RTT;
using namespace RTT::types;

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

bool orogen_typekits::<%= typekit.name %>TypekitPlugin::loadOperators()
{
	return true;
}
bool orogen_typekits::<%= typekit.name %>TypekitPlugin::loadConstructors()
{
    RTT::types::TypeInfoRepository::shared_ptr ti_repository = RTT::types::TypeInfoRepository::Instance();

    // Add sequence constructors for sequence types:
    <% registered_types.each do |type| if type.info_type == "RTT::types::SequenceTypeInfo" %>
    ti_repository->type( "<%= type.full_name %>" )->addConstructor( newConstructor(sequence_ctor< <%= type.deference.cxx_name %> >()));
    ti_repository->type( "<%= type.full_name %>" )->addConstructor( newConstructor(sequence_ctor2< <%= type.deference.cxx_name %> >()));
    ti_repository->type( "<%= type.full_name %>" )->addConstructor( new SequenceBuilder< <%= type.deference.cxx_name %> >());
    <% end end %>

	return true;
}
std::string orogen_typekits::<%= typekit.name %>TypekitPlugin::getName()
{ return "/orogen/<%= typekit.name %>"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::<%= typekit.name %>TypekitPlugin);

