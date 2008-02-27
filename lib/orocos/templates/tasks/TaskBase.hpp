#ifndef <%= task.name.upcase %>_TASK_BASE_HPP
#define <%= task.name.upcase %>_TASK_BASE_HPP

#include <rtt/TaskContext.hpp>
<% if task.period %>
#include <rtt/PeriodicActivity.hpp>
<% else %>
#include <rtt/NonPeriodicActivity.hpp>
<% end %>

namespace <%= component.name %> {
    class <%= task.name %>Base : public RTT::TaskContext
    {
	<% if task.period %>
	    RTT::PeriodicActivity activity;
	<% else %>
	    RTT::NonPeriodicActivity activity;
	<% end %>

    public:
	<%= task.name %>Base();
    };
}

#endif

