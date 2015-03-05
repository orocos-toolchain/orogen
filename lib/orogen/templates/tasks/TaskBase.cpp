/* Generated from orogen/lib/orogen/templates/tasks/TAskBase.cpp */

#include "tasks/<%= task.basename %>Base.hpp"

using namespace <%= project.name %>;

<% code_before, code_after =
   task.base_implementation_code.partition(&:first)
   code_before.map! { |_, c| c.call }
   code_after.map! { |_, c| c.call }
%>

<%= code_before.sort.join("\n") %>

<% initializer_list = task.self_base_members.
        sort_by { |m| [m.kind, m.name] }.
        map { |m| 
            ret = m.with_indent(4, :initializer)
            if(ret)
                ", " + ret.strip
            else
                  nil
            end
        }.
        compact.join("\n    ") %> 

<%= task.basename %>Base::<%= task.basename %>Base(std::string const& name<%= ", TaskCore::TaskState state" unless task.fixed_initial_state? %>)
<% if task.superclass.fixed_initial_state? %>
    : ::<%= task.superclass.name %>(name)
<% elsif task.needs_configuration? %>
    : ::<%= task.superclass.name %>(name, TaskCore::PreOperational)
<% else %>
    : ::<%= task.superclass.name %>(name, state)
<% end %>
<%= initializer_list %>
{
    setupComponentInterface();
}

<%= task.basename %>Base::<%= task.basename %>Base(std::string const& name, RTT::ExecutionEngine* engine<%= ", TaskCore::TaskState state" unless task.fixed_initial_state? %>)
<% if task.superclass.fixed_initial_state? %>
    : ::<%= task.superclass.name %>(name, engine)
<% elsif task.needs_configuration? %>
    : ::<%= task.superclass.name %>(name, engine, TaskCore::PreOperational)
<% else %>
    : ::<%= task.superclass.name %>(name, engine, state)
<% end %>
<%= initializer_list %>
{
    setupComponentInterface();
}

<%= task.basename %>Base::~<%= task.basename %>Base()
{
<%= task.self_base_members.
    sort_by { |m| [m.kind, m.name] }.
    map { |m| m.with_indent(4, :destructor) }.
    compact.join("\n") %>
}

void <%= task.basename %>Base::setupComponentInterface()
{
<%= task.self_base_members.
    sort_by { |m| [m.kind, m.name] }.
    map { |m| m.with_indent(4, :constructor) }.
    compact.join("\n") %>

    <% if task.extended_state_support? %>
    _state.keepLastWrittenValue(true);
    _state.write(getTaskState());
    <% end %>
}

<%= task.self_base_methods.
        sort_by { |m| [m.name] }.
        map { |m| m.with_indent(0, :definition) }.
        compact.join("\n") %>

<% if task.extended_state_support? %>
void <%= task.basename %>Base::report(States state)
{
    _state.write(state);
}
void <%= task.basename %>Base::state(States state)
{
    _state.write(state);
}
void <%= task.basename %>Base::error(States state)
{
    _state.write(state);
    TaskContext::error();
}
void <%= task.basename %>Base::exception(States state)
{
    _state.write(state);
    TaskContext::exception();
}
void <%= task.basename %>Base::fatal(States state)
{
    _state.write(state);
    TaskContext::fatal();
}
<%= task.basename %>Base::States <%= task.basename %>Base::state() const
{
    return static_cast<<%= task.basename %>Base::States>(_state.getLastWrittenValue());
}
<% end %>

<% if task.extended_state_support? && !task.superclass.extended_state_support? %>
struct StateExporter
{
    RTT::TaskContext const& task;
    RTT::OutputPort<<%= task.project.find_type('/int32_t').cxx_name %>>&   port;

    StateExporter(RTT::TaskContext const& task, RTT::OutputPort<int>& port)
        : task(task), port(port) {}
    ~StateExporter()
    {
        port.write(task.getTaskState());
    }
};
<% end %>

bool <%= task.basename %>Base::start()
{
<% if task.extended_state_support? && !task.superclass.extended_state_support? %>
    StateExporter exporter(*this, _state);
<% end %>
    bool started = <%= superclass.name %>::start();
    if (!started)
        return false;

    <% task.self_ports.find_all { |p| p.kind_of?(InputPort) }.each do |port| %>
    _<%= port.name %>.clear();
    <% end %>
    return true;
}

<% if task.extended_state_support? && !task.superclass.extended_state_support? %>
bool <%= task.basename %>Base::configure()
{
    StateExporter exporter(*this, _state);
    return <%= superclass.name %>::configure();
}
bool <%= task.basename %>Base::recover()
{
    StateExporter exporter(*this, _state);
    return <%= superclass.name %>::recover();
}
bool <%= task.basename %>Base::stop()
{
    StateExporter exporter(*this, _state);
    return <%= superclass.name %>::stop();
}
bool <%= task.basename %>Base::cleanup()
{
    StateExporter exporter(*this, _state);
    return <%= superclass.name %>::cleanup();
}
void <%= task.basename %>Base::fatal()
{ return fatal(FATAL_ERROR); }
void <%= task.basename %>Base::error()
{ return error(RUNTIME_ERROR); }
void <%= task.basename %>Base::exception()
{ return exception(EXCEPTION); }
<% end %>

<% task.base_hook_code.keys.sort.each do |hook_name| %>
<%    snippets = task.base_hook_code[hook_name] %>
<%    next if snippets.empty? %>
<%    is_boolean = (hook_name == "start" || hook_name == "configure") %>
<%= (is_boolean ? 'bool' : 'void') %> <%= task.basename %>Base::<%= hook_name %>Hook()
{
    <% if is_boolean %>
    if (! <%= task.superclass.name %>::<%= hook_name %>Hook())
        return false;
    <% else %>
    <%= task.superclass.name %>::<%= hook_name %>Hook();
    <% end %>

    <% snippets.each do |code| %>
        <% if code.respond_to?(:to_str) %>
<%= code %>
        <% else %>
<%= code.call %>
        <% end %>
    <% end %>

    <% if is_boolean %>
    return true;
    <% end %>
}
<% end %>

<%= code_after.join("\n") %>

