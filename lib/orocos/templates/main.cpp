#include <rtt/os/main.h>
<% if component.toolkit %>#include "toolkit/<%= component.toolkit.name %>Toolkit.hpp"<% end %>
<% component.used_toolkits.each do |name| %>
#include <toolkit/<%= name %>Toolkit.hpp>
<% end %>
<% deployer.task_activities.each do |task|
    if task.context.external_definition?
        library_name, name = task.context.name.split("::") %>
#include <<%= File.join(library_name.downcase, name) %>.hpp>
    <% else %>
#include <tasks/<%= task.context.name %>.hpp>
    <% end %>
<% end %>
<% if deployer.corba_enabled? %>
#include <rtt/corba/ControlTaskServer.hpp>
using RTT::Corba::ControlTaskServer;
<% end %>

<% require 'set'
    activity_types = deployer.task_activities.
    map { |t| t.activity_type }.
    to_set %>
<% activity_types.each do |activity_klassname| %>
#include <rtt/<%= activity_klassname%>.hpp>
<% end %>

<% if deployer.browse %>
#include <ocl/TaskBrowser.hpp>
<% end %>

using namespace Orocos;
int ORO_main(int argc, char* argv[])
{
   <% if deployer.loglevel %>
   if ( log().getLogLevel() < Logger::<%= deployer.loglevel %> ) {
       log().setLogLevel( Logger::<%= deployer.loglevel %> );
   }
   <% end %>

   <% if component.toolkit %>RTT::Toolkit::Import( <%= component.toolkit.name %>::Toolkit );<% end %>
   <% component.used_toolkits.each do |name| %>
   RTT::Toolkit::Import( <%= name %>::Toolkit );
   <% end %>

<% if deployer.corba_enabled? %>
    ControlTaskServer::InitOrb(argc, argv);
<% end %>

    TaskContext* main_task = 0;
<% deployer.task_activities.each do |task| %>
    <%= task.context.class_name %> task_<%= task.name%>("<%= task.name %>");
    RTT::<%= task.activity_type %> activity_<%= task.name%>(
            <%= task.rtt_scheduler %>,
            <%= task.rtt_priority %>,
            <% if task.period %><%= task.period %>, <% end %>
            task_<%= task.name%>.engine());
    <% if task.period %>
    RTT::OS::PeriodicThread* thread_<%= task.name %> =
        dynamic_cast<RTT::OS::PeriodicThread*>(activity_<%= task.name %>.thread());
    thread_<%= task.name %>->setMaxOverrun(<%= task.max_overruns %>);
    <% end %>
    <% task.properties.each do |prop|
        if prop.value %>
    task_<%= task.name %>.properties()->getProperty<<%= prop.interface_object.type.full_name('::', true) %>>("<%= prop.name %>")->set(<%= prop.value.inspect %>);
        <% end %>
    <% end %>

    <% if deployer.corba_enabled? %>
    ControlTaskServer::Create( &task_<%= task.name%> );
    <% end %>
<% end %>

<% reporter_id = 0 %>
<% if !deployer.file_reporters.empty?
        deployer.file_reporters.each do |filename, (reporter_activity, method_calls)|
            method_calls.each do |type, reported_activity, args| %>
                task_<%= reporter_activity.name %>.connectPeers(&task_<%= reported_activity.name %>);
                task_<%= reporter_activity.name %>.report<%= type %>(<%= args.map { |v| "\"#{v}\"" }.join(", ") %>);
            <% end %>
        <% end %>
<% end %>

    // Start some activities
<% deployer.task_activities.each do |task|
    if task.start?
        if task.context.initial_state == 'PreOperational' %>
    if (!task_<%= task.name %>.configure())
    {
        log(Error) << "cannot configure <%= task.name %>" << endlog();
        return -1;
    }
        <% end %>
    if (!task_<%= task.name %>.start())
    {
        log(Error) << "cannot start <%= task.name %>" << endlog();
        return -1;
    }
    <% end %>
<% end %>

<% deployer.connections.each do |src, dst| %>
    PortInterface* src = task_<% src.activity.name %>.ports()->getPort("<%= src.name %>");
    PortInterface* dst = task_<% dst.activity.name %>.ports()->getPort("<%= dst.name %>");
    src->connectTo(dst);
<% end %>

<% if deployer.corba_enabled? %>
    ControlTaskServer::RunOrb();
    ControlTaskServer::DestroyOrb();
<% elsif deployer.browse %>
    TaskBrowser browser(& task_<%= deployer.browse.name %>);
    browser.loop();
<% end %>

    // Kill all running activities
<% deployer.task_activities.each do |task| %>
    activity_<%= task.name%>.stop();
<% end %>

    return 0;
}

