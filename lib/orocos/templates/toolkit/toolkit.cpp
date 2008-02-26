#include <rtt/Types.hpp>
#include <rtt/TemplateTypeInfo.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Toolkit.hpp>
#include "<%= name %>Toolkit.hpp"
#include "<%= name %>ToolkitTypes.hpp"
<% if corba_enabled? %>
#include <rtt/corba/CorbaTemplateProtocol.hpp>
#include "<%= name %>ToolkitCorba.hpp"
<% end %>

using namespace <%= name %>;
using RTT::Property;
using RTT::PropertyBag;
using RTT::TypeInfoRepository;

namespace <%= name %> {
    <% generated_types.each do |type| %>
<%= Orocos::Generation.render_template 'toolkit/type_info.cpp', binding %>
    <% end %>
}

std::string ToolkitPlugin::getName() { return "<%= name %>"; }
bool ToolkitPlugin::loadTypes()
{
    TypeInfoRepository::shared_ptr ti_repository = TypeInfoRepository::Instance();
    RTT::TypeInfo* ti;

    <% generated_types.each do |type| %>ti = new <%= type.basename %>TypeInfo();
	<% if corba_enabled? %>ti->addProtocol(ORO_CORBA_PROTOCOL_ID, new RTT::detail::CorbaTemplateProtocol< <%= name %>::<%= type.basename %> >());<% end %>
	ti_repository->addType( ti );<% end %>
	
    return true;
}

bool ToolkitPlugin::loadOperators() { return true; }
bool ToolkitPlugin::loadConstructors() { return true; }

ToolkitPlugin <%= name %>::Toolkit;
