#ifndef <%= task.name.upcase %>_TASK_BASE_HPP
#define <%= task.name.upcase %>_TASK_BASE_HPP

#include <rtt/TaskContext.hpp>
<% if component.toolkit %>#include "<%= component.name %>ToolkitTypes.hpp"<% end %>

<% unless task.methods.empty? %>#include <rtt/Method.hpp><% end %>
<% unless task.commands.empty? %>#include <rtt/Command.hpp><% end %>
<% if task.period %>#include <rtt/PeriodicActivity.hpp>
<% else %>#include <rtt/NonPeriodicActivity.hpp><% end %>

namespace <%= component.name %> {
    class <%= task.name %>;
    class <%= task.name %>Base : public RTT::TaskContext
    {
        <% if task.period %>RTT::PeriodicActivity<% else %>RTT::NonPeriodicActivity<% end %> _activity;
	<%= task.name %>& _self;

	<% unless task.properties.empty? %>/** Properties */<% end %>
    <% task.properties.each do |prop| %>
	RTT::Property< <%= prop.type.full_name('::', true) %> > _<%= prop.name %>;
    <% end %>

	<% unless task.methods.empty? %>/** Methods */<% end %>
    <% task.methods.each do |meth| %>
	RTT::Method< <%= meth.signature(false) %> > _<%= meth.name %>;
    <% end %>

	<% unless task.commands.empty? %>/** Commands */<% end %>
    <% task.commands.each do |cmd| %>
	RTT::Command< bool<%= cmd.work_signature(false) %> > _<%= cmd.name %>;
    <% end %>

    public:
	<%= task.name %>Base();
    };
}

#endif

