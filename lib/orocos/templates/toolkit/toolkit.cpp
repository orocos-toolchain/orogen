#include <rtt/Types.hpp>
#include <rtt/TemplateTypeInfo.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Toolkit.hpp>
#include "<%= name %>Toolkit.hpp"
#include "<%= name %>ToolkitTypes.hpp"

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
    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    <% generated_types.each do |type| %>ti->addType( new <%= type.basename %>TypeInfo() );<% end %>
    return true;
}

bool ToolkitPlugin::loadOperators() { return true; }
bool ToolkitPlugin::loadConstructors() { return true; }

ToolkitPlugin <%= name %>::Toolkit;
