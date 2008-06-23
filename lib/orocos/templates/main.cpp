#include <rtt/os/main.h>
<% if component.toolkit %>#include "toolkit/<%= toolkit.name %>Toolkit.hpp"<% end %>
<% component.used_toolkits.each do |name| %>
#include <toolkit/<%= name %>Toolkit.hpp>
<% end %>
<% component.tasks.each do |task| %>
#include <tasks/<%= task.name %>.hpp>
<% end %>
<% if component.corba_enabled? %>
#include <rtt/corba/ControlTaskServer.hpp>
using RTT::Corba::ControlTaskServer;
<% end %>

#include <rtt/PeriodicActivity.hpp>
#include <rtt/NonPeriodicActivity.hpp>


int ORO_main(int argc, char* argv[])
{
   <% if component.toolkit %>RTT::Toolkit::Import( <%= component.toolkit.name %>::Toolkit );<% end %>
   <% component.used_toolkits.each do |name| %>
   RTT::Toolkit::Import( <%= name %>::Toolkit );
   <% end %>

<% if component.corba_enabled? %>
    ControlTaskServer::InitOrb(argc, argv);
<% end %>

<% task_activities.each do |task| %>
{
    <%= component.name %>::<%= task.context.name %> task;
    <% if task.period %>RTT::PeriodicActivity<% else %>RTT::NonPeriodicActivity<% end %>
        activity(<%= task.rtt_scheduler %>, <%= task.rtt_priority %>, <% if task.period %><%= task.period %>, <% end %>task.engine());

    <% if component.corba_enabled? %>
    ControlTaskServer::Create( &task );
    <% end %>
}
<% end %>

<% if component.corba_enabled? %>
    ControlTaskServer::RunOrb();
    ControlTaskServer::DestroyOrb();
<% end %>

    return 0;
}

