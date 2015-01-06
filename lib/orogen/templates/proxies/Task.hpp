/* Generated from orogen/lib/orogen/templates/proxies/Task.hpp */

#ifndef <%= component.name.upcase %>_<%= task.basename.upcase %>_PROXY_TASK_HPP
#define <%= component.name.upcase %>_<%= task.basename.upcase %>_PROXY_TASK_HPP


#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <<%= task.superclass.header_file %>>
<% if !task.new_operations.empty? %>
#include <rtt/Operation.hpp>
<% end %>
<% if !task.all_ports.empty? %>
#include <rtt/Port.hpp>
#include <orocos_cpp/ProxyPort.hpp>
<% end %>
<% if task.extended_state_support? %>
#include <<%= component.typekit.name %>/TaskStates.hpp>
<% end %>


<% task.used_typekits.sort_by(&:name).each do |tk| %>
  <% next if tk.virtual? %>
#include <<%= tk.name %>/typekit/Types.hpp>
<% end %>
<% task.implemented_classes.sort.each do |class_name, include_file| %>
#include <<%= include_file %>> // to get <%= class_name %>
<% end %>
<% if component.typekit %>
#include "<%= component.typekit.name %>/typekit/Types.hpp"
<% end %>


namespace <%= component.name %> {

namespace proxies {
    
class <%= task.basename %> : public RTT::corba::TaskContextProxy
{
    protected:

    public:
        /** TaskContext constructor for <%= task.basename %>
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        <%= task.basename %>(std::string location, bool is_ior);

        /** Default deconstructor of <%= task.basename %>
         */
        ~<%= task.basename %>();
        
        void synchronize();
        
        static void initTypes();
<%= 
result = ""
task.each_input_port do |port|
    result << "        InputProxyPort< #{port.type.cxx_name} > #{port.name};\n"
end

task.each_output_port do |port|
    result << "        OutputProxyPort< #{port.type.cxx_name} > #{port.name};\n"    
end

result

%>

<%= 
result = ""
task.each_property do |property|
    result << "        RTT::Property< #{property.type.cxx_name} > &#{property.name};\n"
end
result
%>

<%= 
result = ""
task.each_operation do |operation|
    result << "        #{operation.signature};\n"
end
result
%>

};
}
}
#endif // TASK_H
