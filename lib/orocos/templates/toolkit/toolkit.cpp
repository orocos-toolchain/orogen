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

using RTT::Property;
using RTT::PropertyBag;
using RTT::TypeInfoRepository;

<% generated_types.each do |type| 
      code = Generation.adapt_namespace(namespace, type.namespace)
      namespace = type.namespace %>
<%= code %>
<%= Orocos::Generation.render_template 'toolkit/type_info.cpp', binding %>
<% end %>
<%= Generation.adapt_namespace(namespace, '/') %>

namespace <%= name %> {
    std::string ToolkitPlugin::getName() { return "<%= name %>"; }
    bool ToolkitPlugin::loadTypes()
    {
	TypeInfoRepository::shared_ptr ti_repository = TypeInfoRepository::Instance();
	RTT::TypeInfo* ti;

	<% generated_types.each do |type| %>ti = new <%= type.full_name('::') %>TypeInfo();
	<% if corba_enabled? %>ti->addProtocol(ORO_CORBA_PROTOCOL_ID, new RTT::detail::CorbaTemplateProtocol< <%= type.full_name('::') %> >());<% end %>
	ti_repository->addType( ti );<% end %>
	    
	return true;
    }

    bool ToolkitPlugin::loadOperators() { return true; }
    bool ToolkitPlugin::loadConstructors() { return true; }

    ToolkitPlugin Toolkit;
}
