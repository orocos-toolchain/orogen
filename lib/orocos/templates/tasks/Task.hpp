#ifndef <%= task.name.upcase %>_TASK_HPP
#define <%= task.name.upcase %>_TASK_HPP

#include "tasks/<%= task.name %>Base.hpp"

namespace <%= component.name %> {
    class <%= task.name %> : public <%= task.name %>Base
    {
	friend class <%= task.name %>Base;
    protected:
    <% task.methods.each do |meth| %>
	<%= meth.signature.gsub('(', " #{meth.method_name}(") %>;
    <% end %>
    <% task.commands.each do |cmd| %>
	bool <%= cmd.work_method_name %><%= cmd.work_signature %>;
	bool <%= cmd.completion_method_name %><%= cmd.completion_signature %>;
    <% end %>

    public:
        <%= task.name %>(std::string const& name = "<%= task.name %>");
    };
}

#endif

