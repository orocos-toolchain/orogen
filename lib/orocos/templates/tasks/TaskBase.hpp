#ifndef <%= task.name.upcase %>_TASK_BASE_HPP
#define <%= task.name.upcase %>_TASK_BASE_HPP

<% unless task.methods.empty? %>
#include <rtt/Method.hpp>
<% end %>
#include <rtt/TaskContext.hpp>
<% if task.period %>
#include <rtt/PeriodicActivity.hpp>
<% else %>
#include <rtt/NonPeriodicActivity.hpp>
<% end %>
#include "<%= component.name %>ToolkitTypes.hpp"

namespace <%= component.name %> {
    class <%= task.name %>;
    class <%= task.name %>Base : public RTT::TaskContext
    {
        <% if task.period %>RTT::PeriodicActivity<% else %>RTT::NonPeriodicActivity<% end %> _activity;
	<%= task.name %>& _self;

	<% unless task.properties.empty? %>/** Properties */<% end %>
    <% task.properties.each do |prop| %>
	RTT::Property< <%= prop.type.full_name('::') %> > _<%= prop.name %>;
    <% end %>

	<% unless task.methods.empty? %>/** Methods */<% end %>
    <% task.methods.each do |meth| %>
	RTT::Method< <%= meth.signature %> > _<%= meth.name %>;
    <% end %>

    public:
	<%= task.name %>Base();
    };
}

#endif

