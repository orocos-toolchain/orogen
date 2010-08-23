#include <rtt/os/main.h>

#include <rtt/typekit/RealTimeTypekit.hpp>
<% deployer.rtt_transports.each do |transport_name| %>
#include <rtt/transports/<%= transport_name %>/TransportPlugin.hpp>
<% end %>

<% if component.typekit || !component.used_typekits.empty? %>#include <rtt/types/TypekitPlugin.hpp><% end %>
<% if typekit = component.typekit %>
#include "typekit/Plugin.hpp"
<% deployer.transports.each do |transport_name| %>
#include "typekit/transports/<%= transport_name %>/TransportPlugin.hpp"
<% end %>
<% end %>
<% deployer.used_typekits.each do |tk| %>
#include <<%= tk.name %>/typekit/Plugin.hpp>
    <% deployer.transports.each do |transport_name| %>
#include <<%= tk.name %>/transports/<%= transport_name %>/TransportPlugin.hpp>
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
#include <signal.h>
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

void sigint_quit_orb(int)
{
    RTT::corba::TaskContextServer::ShutdownOrb(false);
}

using namespace RTT;
int ORO_main(int argc, char* argv[])
{
   <% if deployer.loglevel %>
   if ( log().getLogLevel() < Logger::<%= deployer.loglevel %> ) {
       log().setLogLevel( Logger::<%= deployer.loglevel %> );
   }
   <% end %>

   RTT::types::TypekitRepository::Import( new RTT::types::RealTimeTypekitPlugin );
   <% if deployer.transports.include?('corba') %>
   RTT::types::TypekitRepository::Import( new RTT::corba::CorbaLibPlugin );
   <% end %>

   <% if component.typekit %>
   RTT::types::TypekitRepository::Import( new orogen_typekits::<%= component.name %>TypekitPlugin );
   <% deployer.transports.each do |transport_name| %>
   RTT::types::TypekitRepository::Import( new orogen_typekits::<%= component.name %><%= transport_name.capitalize %>TransportPlugin );
   <% end %>
   <% end %>
   <% deployer.used_typekits.each do |tk| %>
   RTT::types::TypekitRepository::Import( new orogen_typekits::<%= tk.name %>TypekitPlugin );
       <% deployer.transports.each do |transport_name| %>
   RTT::types::TypekitRepository::Import( new orogen_typekits::<%= tk.name %><%= transport_name.capitalize %>TransportPlugin );
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
    task_<%= task.name %>.properties()->getProperty<<%= prop.interface_object.type.cxx_name %>>("<%= prop.name %>")->set(<%= prop.value.inspect %>);
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
            base::OutputPortInterface* src = dynamic_cast<base::OutputPortInterface*>(
                    task_<%= src.activity.name %>.ports()->getPort("<%= src.name %>"));
            base::InputPortInterface* dst = dynamic_cast<base::InputPortInterface*>(
                    task_<%= dst.activity.name %>.ports()->getPort("<%= dst.name %>"));
            src->createConnection(*dst, policy);
        }
        <% end %>
    <% end %>

<% if deployer.corba_enabled? %>
    struct sigaction sigint_handler;
    sigint_handler.sa_handler = &sigint_quit_orb;
    sigemptyset(&sigint_handler.sa_mask);
    sigint_handler.sa_flags     = 0;
    sigint_handler.sa_restorer  = 0;
    if (-1 == sigaction(SIGINT, &sigint_handler, 0))
    {
        std::cerr << "failed to install SIGINT handler" << std::endl;
        return 1;
    }
    sigset_t unblock_sigint;
    sigemptyset(&unblock_sigint);
    sigaddset(&unblock_sigint, SIGINT);
    if (-1 == sigprocmask(SIG_UNBLOCK, &unblock_sigint, NULL))
    {
        std::cerr << "failed to install SIGINT handler" << std::endl;
        return 1;
    }
    RTT::corba::TaskContextServer::RunOrb();
<% elsif deployer.browse %>
    OCL::TaskBrowser browser(& task_<%= deployer.browse.name %>);
    browser.loop();
<% end %>

    return 0;
}

