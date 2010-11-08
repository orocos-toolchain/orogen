/* Generated from orogen/lib/orogen/templates/tasks/TAskBase.cpp */

#include "tasks/<%= task.basename %>Base.hpp"

using namespace <%= component.name %>;

<%= task.basename %>Base::<%= task.basename %>Base(std::string const& name<%= ", TaskCore::TaskState state" unless task.fixed_initial_state? %>)
<% if task.superclass.fixed_initial_state? %>
    : ::<%= task.superclass.name %>(name)
<% elsif task.needs_configuration? %>
    : ::<%= task.superclass.name %>(name, TaskCore::PreOperational)
<% else %>
    : ::<%= task.superclass.name %>(name, state)
<% end %>
    <% task.self_properties.each do |prop| %>
    , _<%= prop.name %>("<%= prop.name %>", "<%= prop.doc %>")<% end %>
    <% task.self_attributes.each do |att| %>
    , _<%= att.name %>("<%= att.name %>")<% end %>
    <% task.self_ports.each do |port| %>
    , _<%= port.name %>("<%= port.name %>")<% end %>
    <% task.new_operations.each do |op| %>
    , _<%= op.name %>("<%= op.name %>", &<%= task.basename %>Base::<%= op.method_name %>, this, <%= if op.in_caller_thread then "RTT::ClientThread" else "RTT::OwnThread" end %>)<% end %>

    <% if task.superclass.name == "RTT::TaskContext" %>
    , _getModelName("getModelName", &<%= task.basename %>Base::getModelName, this, RTT::ClientThread)
    <% end %>
{
    <% task.self_properties.each do |prop|
        if prop.default_value %>
        _<%= prop.name %>.set(<%= prop.cxx_default_value %>);
        <% end %>
    properties()->addProperty( _<%= prop.name %> );<% end %>
    <% task.self_attributes.each do |att|
        if att.default_value %>
        _<%= att.name %>.set(<%= att.cxx_default_value %>);
        <% end %>
    attributes()->addAttribute( _<%= att.name %> );<% end %>
    <% (task.self_ports - task.event_ports).each do |port| %>
    ports()->addPort( _<%= port.name %> );<% end %>
    <% (task.event_ports & task.self_ports).each do |port| %>
    ports()->addEventPort( _<%= port.name %> );<% end %>
    <% task.new_operations.each do |op| 
	argument_setup = op.arguments.
	    map { |n, _, d| ", \"#{n}\", \"#{d}\"" }.
	    join("")
    %>
    provides()->addOperation( _<%= op.name %>);<% end %>
    <% if task.superclass.name == "RTT::TaskContext" %>
    provides()->addOperation( _getModelName );
    <% end %>

    <% if task.extended_state_support? %>
    _state.keepLastWrittenValue(true);
    _state.write(getTaskState());
    <% end %>
}

std::string <%= task.basename %>Base::getModelName() const
{
    return "<%= task.name %>";
}

<% if task.extended_state_support? %>
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
    RTT::OutputPort<int>&   port;

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
{ return fatal(EXCEPTION); }
<% end %>

