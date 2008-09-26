#ifndef <%= component.name.upcase %>_<%= task.basename.upcase %>_TASK_HPP
#define <%= component.name.upcase %>_<%= task.basename.upcase %>_TASK_HPP

#include "<%= component.name.downcase %>/<%= task.basename %>Base.hpp"

namespace <%= component.name %> {
    class <%= task.basename %> : public <%= task.basename %>Base
    {
	friend class <%= task.basename %>Base;
    protected:
    <% task.methods.each do |meth| %>
	<%= meth.signature.gsub('(', " #{meth.method_name}(") %>;
    <% end %>
    <% task.commands.each do |cmd| %>
	bool <%= cmd.work_method_name %><%= cmd.work_signature %>;
	bool <%= cmd.completion_method_name %><%= cmd.completion_signature %>;
    <% end %>

    public:
        <%= task.basename %>(std::string const& name = "<%= task.name %>");
    };
}

#endif

