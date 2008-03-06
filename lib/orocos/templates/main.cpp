#include <rtt/os/main.h>
<% if component.toolkit %>#include "toolkit/<%= toolkit.name %>Toolkit.hpp"<% end %>
<% component.tasks.each do |task| %>
#include <tasks/<%= task.name %>.hpp>
<% end %>
<% if component.corba_enabled? %>
#include <rtt/corba/ControlTaskServer.hpp>
using RTT::Corba::ControlTaskServer;
<% end %>


int ORO_main(int argc, char* argv[])
{
   <% if component.toolkit %>RTT::Toolkit::Import( <%= component.toolkit.name %>::Toolkit );<% end %>

<% if component.corba_enabled? %>
    ControlTaskServer::InitOrb(argc, argv);
<% end %>

<% component.tasks.each do |task| %>
    <%= component.name %>::<%= task.name %> <%= task.name %>;
    ControlTaskServer::Create( &<%= task.name %> );
<% end %>

<% if component.corba_enabled? %>
    ControlTaskServer::RunOrb();

    ControlTaskServer::DestroyOrb();
<% end %>

    return 0;
}

