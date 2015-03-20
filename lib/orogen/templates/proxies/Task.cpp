#include "<%= task.basename %>.hpp"


#include <rtt/typekit/RealTimeTypekit.hpp>
<% if component.enabled_transports.include?('corba') %>
#include <rtt/transports/corba/TransportPlugin.hpp>
<% end %>
<% if component.enabled_transports.include?('mqueue') %>
#include <rtt/transports/mqueue/TransportPlugin.hpp>
<% end %>

<% if component.typekit || !component.used_typekits.empty? %>
#include <rtt/types/TypekitPlugin.hpp>
<% end %>
<% if typekit = component.typekit %>
#include <<%= typekit.name %>/typekit/Plugin.hpp>
<% component.enabled_transports.each do |transport_name| %>
#include "typekit/transports/<%= transport_name %>/TransportPlugin.hpp"
<% end %>
<% end %>
<% component.used_typekits.each do |tk| %>
    <% next if tk.virtual? %>
    <% next if tk.name == "logger" %>
#include <<%= tk.name %>/typekit/Plugin.hpp>
    <% component.enabled_transports.each do |transport_name| %>
#include <<%= tk.name %>/transports/<%= transport_name %>/TransportPlugin.hpp>
    <% end %>
<% end %>

namespace <%= component.name %> {

namespace proxies {

<%= task.basename %>Initializer::<%= task.basename %>Initializer(std::string location, bool is_ior)
:TaskContextProxy()
{
    initTypes();
    
    initFromURIOrTaskname(location, is_ior);
}

void <%= task.basename %>Initializer::initTypes()
{
    RTT::types::TypekitRepository::Import( new RTT::types::RealTimeTypekitPlugin );
    <% if component.enabled_transports.include?('corba') %>
    RTT::types::TypekitRepository::Import( new RTT::corba::CorbaLibPlugin );
    <% end %>
    <% if component.enabled_transports.include?('mqueue') %>
    RTT::types::TypekitRepository::Import( new RTT::mqueue::MQLibPlugin );
    <% end %>

    <% if component.typekit %>
    RTT::types::TypekitRepository::Import( new orogen_typekits::<%= component.name %>TypekitPlugin );
        <% component.enabled_transports.each do |transport_name| %>
    RTT::types::TypekitRepository::Import( new <%= typekit.transport_plugin_name(transport_name) %> );
        <% end %>
    <% end %>
    <% component.used_typekits.each do |tk| %>
        <% next if tk.virtual? %>
        <% next if tk.name == "logger" %>

   RTT::types::TypekitRepository::Import( new orogen_typekits::<%= tk.name %>TypekitPlugin );
       <% component.enabled_transports.each do |transport_name| %>
   RTT::types::TypekitRepository::Import( new <%= Orocos::Generation::Typekit.transport_plugin_name(transport_name, tk.name) %> );
       <% end %>
   <% end %>
}
    
<%= task.basename %>::<%= task.basename %>(std::string location, bool is_ior) :
<%= 
result = task.basename + "Initializer(location, is_ior),\n"
task.each_port do |port|
    result << "#{port.name}(getPort(\"#{port.name}\")),\n"
end

task.each_property do |property|
    result << "#{property.name}(*(dynamic_cast<RTT::Property< #{property.type.cxx_name} > *>(getProperty(\"#{property.name}\")))),\n"
end

result.slice!(0, result.length() -2)
%>
{
    initTypes();
    
    try {
        initFromURIOrTaskname(location, is_ior);
    } catch (...)
    {
        throw std::runtime_error("Error : Failed to lookup task context " + location);
    }
}

void <%= task.basename %>::synchronize()
{
        RTT::corba::TaskContextProxy::synchronize();
}

<%= 
result = ""
task.each_operation do |operation|
    signature = operation.signature(true) do
        "#{task.basename}::#{operation.name}"
    end 
    result << signature
    result << "{\n"
    result << "RTT::OperationInterfacePart *opIfac = getOperation(\"#{operation.name}\");\n"
    result << "RTT::OperationCaller< #{operation.signature(false)} >  caller(opIfac);\n"
    result << "return caller("
    first = true
    operation.arguments.each do |arg|
        if(first)
            first = false
        else
            result << ", "
        end
        result << " #{arg[0]}"
    end
    result << ");\n"
    result << "}\n"
end
result
%>        
        
}
}