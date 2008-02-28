#ifndef <%= task.name.upcase %>_TASK_BASE_HPP
#define <%= task.name.upcase %>_TASK_BASE_HPP

#include <rtt/TaskContext.hpp>
<% if task.period %>
#include <rtt/PeriodicActivity.hpp>
<% else %>
#include <rtt/NonPeriodicActivity.hpp>
<% end %>
#include "<%= component.name %>ToolkitTypes.hpp"

namespace <%= component.name %> {
    class <%= task.name %>Base : public RTT::TaskContext
    {
	<% if task.period %>
	    RTT::PeriodicActivity activity;
	<% else %>
	    RTT::NonPeriodicActivity activity;
	<% end %>

	/** Properties */
	<% task.properties.each do |prop| %>
	    RTT::Property< <%= prop.type.full_name('::') %> > _<%= prop.name %>;
	<% end %>
	    

    public:
	<%= task.name %>Base();
    };
}

#endif

