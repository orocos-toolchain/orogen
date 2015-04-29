#include "<%= task.basename %>.hpp"


#include <rtt/typekit/RealTimeTypekit.hpp>
<% if project.enabled_transports.include?('corba') %>
#include <rtt/transports/corba/TransportPlugin.hpp>
<% end %>
<% if project.enabled_transports.include?('mqueue') %>
#include <rtt/transports/mqueue/TransportPlugin.hpp>
<% end %>

<% if project.typekit || !project.used_typekits.empty? %>
#include <rtt/types/TypekitPlugin.hpp>
<% end %>
<% if typekit = project.typekit %>
#include <<%= typekit.name %>/typekit/Plugin.hpp>
<% project.enabled_transports.each do |transport_name| %>
#include "typekit/transports/<%= transport_name %>/TransportPlugin.hpp"
<% end %>
<% end %>
<% project.used_typekits.each do |tk| %>
    <% next if tk.virtual? %>
    <% next if tk.name == "logger" %>
#include <<%= tk.name %>/typekit/Plugin.hpp>
    <% project.enabled_transports.each do |transport_name| %>
#include <<%= tk.name %>/transports/<%= transport_name %>/TransportPlugin.hpp>
    <% end %>
<% end %>

namespace <%= project.name %> {

namespace proxies {

<%= task.basename %>Initializer::<%= task.basename %>Initializer(std::string location, bool is_ior)
:TaskContextProxy()
{
    initTypes();
    
    try {
        initFromURIOrTaskname(location, is_ior);
    } catch (...)
    {
        throw std::runtime_error("Error : Failed to lookup task context " + location);
    }
}

void <%= task.basename %>Initializer::initTypes()
{
    RTT::types::TypekitRepository::Import( new RTT::types::RealTimeTypekitPlugin );
    <% if project.enabled_transports.include?('corba') %>
    RTT::types::TypekitRepository::Import( new RTT::corba::CorbaLibPlugin );
    <% end %>
    <% if project.enabled_transports.include?('mqueue') %>
    RTT::types::TypekitRepository::Import( new RTT::mqueue::MQLibPlugin );
    <% end %>

    <% if project.typekit  %>
    RTT::types::TypekitRepository::Import( new orogen_typekits::<%= project.name %>TypekitPlugin );
        <% project.enabled_transports.each do |transport_name| %>
    RTT::types::TypekitRepository::Import( new <%= typekit.transport_plugin_name(transport_name) %> );
        <% end %>
    <% end %>
    <% project.used_typekits.each do |tk| %>
        <% next if tk.virtual? %>
        <% next if tk.name == "logger" %>

   RTT::types::TypekitRepository::Import( new orogen_typekits::<%= tk.name %>TypekitPlugin );
       <% project.enabled_transports.each do |transport_name| %>
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

task.each_property.to_a.uniq{|s| s.name}.each do |property|
    result << "#{property.name}(*(dynamic_cast<RTT::Property< #{property.type.cxx_name} > *>(getProperty(\"#{property.name}\")))),\n"
end

result.slice!(0, result.length() -2)
%>
{
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