#include <rtt/os/main.h>
<% if component.typekit || !component.used_typekits.empty? %>#include <rtt/Typekit.hpp><% end %>
<% if component.typekit %>
#include "typekit/<%= component.name %>Typekit.hpp"
<% if deployer.corba_enabled? %>
#include "typekit/<%= component.name %>TypekitCorba.hpp"
<% end %>
<% end %>
<% deployer.used_typekits.each do |tk| %>
#include <typekit/<%= tk.name %>Typekit.hpp>
    <% if deployer.corba_enabled? %>
#include <typekit/<%= tk.name %>TypekitCorba.hpp>
    <% end %>
<% end %>

<% task_activities = deployer.task_activities.
        sort_by(&:name) %>

<% task_activities.each do |task| %>
#include <<%= task.context.header_file %>>
<% end %>
<% if deployer.corba_enabled? %>
#include <rtt/transports/corba/ApplicationServer.hpp>
#include <rtt/transports/corba/TaskContextServer.hpp>
<% end %>

<% require 'set'
    activity_headers = task_activities.
        map { |t| t.activity_type.header }.
        compact.
        to_set %>
<% activity_headers.to_a.sort.each do |header| %>
#include <<%= header %>>
<% end %>

<% if deployer.browse %>
#include <ocl/TaskBrowser.hpp>
<% end %>

#include <rtt/Logger.hpp>
#include <rtt/base/ActivityInterface.hpp>
class Deinitializer
{
    friend Deinitializer& operator << (Deinitializer&, RTT::base::ActivityInterface&);

    std::vector<RTT::base::ActivityInterface*> m_activities;

public:
    ~Deinitializer()
    {
        for (std::vector<RTT::base::ActivityInterface*>::const_iterator it = m_activities.begin();
                it != m_activities.end(); ++it)
        {
            (*it)->stop();
        }
    }
};

Deinitializer& operator << (Deinitializer& deinit, RTT::base::ActivityInterface& activity)
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

   <% if component.typekit %>
   RTT::Typekit::Import( orogen_typekits::<%= component.name %>Typekit );
   <% if deployer.corba_enabled? %>
   RTT::Typekit::Import( orogen_typekits::<%= component.name %>CorbaTransport );
   <% end %>
   <% end %>
   <% deployer.used_typekits.each do |tk| %>
   RTT::Typekit::Import( orogen_typekits::<%= tk.name %>Typekit );
       <% if deployer.corba_enabled? %>
   RTT::Typekit::Import( orogen_typekits::<%= tk.name %>CorbaTransport );
       <% end %>
   <% end %>

<% if deployer.corba_enabled? %>
    RTT::corba::ApplicationServer::InitOrb(argc, argv);
<% end %>

<% task_activities.each do |task| %>
    <%= task.context.class_name %> task_<%= task.name%>("<%= task.name %>");
    <% if task.activity_type %>
    <%= task.activity_type.class_name %>* activity_<%= task.name%> = new <%= task.activity_type.class_name %>(
            <%= task.rtt_scheduler %>,
            <%= task.rtt_priority %>,
            <% if task.period %><%= task.period %>, <% end %>
            task_<%= task.name%>.engine());
    task_<%= task.name %>.setActivity(activity_<%= task.name %>);
    <% if task.period %>
    RTT::os::Thread* thread_<%= task.name %> =
        dynamic_cast<RTT::os::Thread*>(activity_<%= task.name %>->thread());
    thread_<%= task.name %>->setMaxOverrun(<%= task.max_overruns %>);
    <% end %>
    <% else %>
    RTT::base::ActivityInterface* activity_<%= task.name %> = task_<%= task.name %>.getActivity().get();
    <% end # activity_type %>
    <% task.properties.sort_by { |prop| prop.name }.each do |prop|
        if prop.value %>
    task_<%= task.name %>.properties()->getProperty<<%= prop.interface_object.type.full_name('::', true) %>>("<%= prop.name %>")->set(<%= prop.value.inspect %>);
        <% end %>
    <% end %>

    <% if deployer.corba_enabled? %>
    RTT::corba::TaskContextServer::Create( &task_<%= task.name%> );
    <% end %>
<% end %>

<% if !deployer.loggers.empty?
        deployer.loggers.sort_by { |filename, _| filename }.each do |filename, logger|
            logger.config.each do |type, reported_activity, args| %>
                task_<%= logger.task.name %>.connectPeers(&task_<%= reported_activity.name %>);
                task_<%= logger.task.name %>.report<%= type %>(<%= args.map { |v| "\"#{v}\"" }.join(", ") %>);
            <% end %>
        <% end %>
<% end %>

   Deinitializer deinit;

    // Start some activities
<% task_activities.each do |task| %>
    deinit << *activity_<%= task.name%>;

    <% if task.start?
        if task.context.needs_configuration? %>
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

<% deployer.peers.sort_by { |a, b| [a.name, b.name] }.each do |a, b| %>
    task_<%= a.name %>.connectPeers(&task_<%= b.name %>);
<% end %>

<% deployer.connections.
    sort_by { |src, dst, policy| [src.name, dst.name] }.
    each do |src, dst, policy|
        if src.kind_of?(TaskDeployment) %>
            task_<%= src.activity.name %>.connectPorts(&task_<%= dst.activity.name %>);
        <% else %>
        {
            <%= policy.to_code("policy") %>
            OutputPortInterface* src = dynamic_cast<OutputPortInterface*>(
                    task_<%= src.activity.name %>.ports()->getPort("<%= src.name %>"));
            InputPortInterface* dst = dynamic_cast<InputPortInterface*>(
                    task_<%= dst.activity.name %>.ports()->getPort("<%= dst.name %>"));
            src->createConnection(*dst, policy);
        }
        <% end %>
    <% end %>

<% if deployer.corba_enabled? %>
    RTT::corba::TaskContextServer::RunOrb();
<% elsif deployer.browse %>
    OCL::TaskBrowser browser(& task_<%= deployer.browse.name %>);
    browser.loop();
<% end %>

    return 0;
}

