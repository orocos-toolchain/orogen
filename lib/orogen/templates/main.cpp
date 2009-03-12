#include <rtt/os/main.h>
<% if component.toolkit || !component.used_toolkits.empty? %>#include <rtt/Toolkit.hpp><% end %>
<% if component.toolkit %>#include "toolkit/<%= component.name %>Toolkit.hpp"<% end %>
<% component.used_toolkits.each do |name, _| %>
#include <toolkit/<%= name %>Toolkit.hpp>
<% end %>
<% deployer.task_activities.each do |task| %>
#include <<%= task.context.header_file %>>
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

#include <rtt/Logger.hpp>
#include <rtt/ActivityInterface.hpp>
class Deinitializer
{
    friend Deinitializer& operator << (Deinitializer&, RTT::ActivityInterface&);

    std::vector<RTT::ActivityInterface*> m_activities;

public:
    ~Deinitializer()
    {
        for (std::vector<RTT::ActivityInterface*>::const_iterator it = m_activities.begin();
                it != m_activities.end(); ++it)
        {
            (*it)->stop();
        }
    }
};

Deinitializer& operator << (Deinitializer& deinit, RTT::ActivityInterface& activity)
{
    deinit.m_activities.push_back(&activity);
    return deinit;
}

using namespace RTT;
int ORO_main(int argc, char* argv[])
{
   <% if deployer.loglevel %>
   if ( log().getLogLevel() < Logger::<%= deployer.loglevel %> ) {
       log().setLogLevel( Logger::<%= deployer.loglevel %> );
   }
   <% end %>

   <% if component.toolkit %>RTT::Toolkit::Import( <%= component.name %>::Toolkit );<% end %>
   <% component.used_toolkits.each do |name, _| %>
   RTT::Toolkit::Import( <%= name %>::Toolkit );
   <% end %>

<% if deployer.corba_enabled? %>
    ControlTaskServer::InitOrb(argc, argv);
<% end %>

<% deployer.task_activities.each do |task| %>
    <%= task.context.class_name %> task_<%= task.name%>("<%= task.full_name %>");
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

<% if !deployer.file_reporters.empty?
        deployer.file_reporters.each do |filename, logger|
            logger.config.each do |type, peek, reported_activity, args| 
                peek = peek ? "true" : "false" %>

                task_<%= logger.task.name %>.connectPeers(&task_<%= reported_activity.name %>);
                task_<%= logger.task.name %>.report<%= type %>(<%= args.map { |v| "\"#{v}\"" }.join(", ") %>, <%= peek %>);
            <% end %>
        <% end %>
<% end %>
<% if !deployer.data_loggers.empty?
        deployer.data_loggers.each do |filename, logger|
            logger.config.each do |type, peek, reported_activity, args| 
                peek = peek ? "true" : "false" %>

                task_<%= logger.task.name %>.connectPeers(&task_<%= reported_activity.name %>);
                task_<%= logger.task.name %>.report<%= type %>(<%= args.map { |v| "\"#{v}\"" }.join(", ") %>, <%= peek %>);
            <% end %>
        <% end %>
<% end %>

   Deinitializer deinit;

    // Start some activities
<% deployer.task_activities.each do |task| %>
    deinit << activity_<%= task.name%>;

    <% if task.start?
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

<% deployer.peers.each do |a, b| %>
    task_<%= a.name %>.connectPeers(&task_<%= b.name %>);
<% end %>

<% deployer.connections.each do |src, dst, policy|
    if src.kind_of?(TaskDeployment) %>
        task_<%= a.activity.name %>.connectPorts(&task_<%= a.activity.name %>);
    <% else %>
    {
        <%= policy.to_code("policy") %>
        WritePortInterface* src = dynamic_cast<WritePortInterface*>(
                task_<%= src.activity.name %>.ports()->getPort("<%= src.name %>"));
        ReadPortInterface* dst = dynamic_cast<ReadPortInterface*>(
                task_<%= dst.activity.name %>.ports()->getPort("<%= dst.name %>"));
        src->createConnection(*dst, policy);
    }
    <% end %>
<% end %>

<% if deployer.corba_enabled? %>
    ControlTaskServer::RunOrb();
    ControlTaskServer::DestroyOrb();
<% elsif deployer.browse %>
    OCL::TaskBrowser browser(& task_<%= deployer.browse.name %>);
    browser.loop();
<% end %>

    return 0;
}

