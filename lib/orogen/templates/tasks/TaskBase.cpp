#include "tasks/<%= task.basename %>Base.hpp"

using namespace <%= component.name %>;

<%= task.basename %>Base::<%= task.basename %>Base(std::string const& name<%= ", TaskCore::TaskState state" unless task.fixed_initial_state? %>)
<% if task.superclass.fixed_initial_state? %>
    : <%= task.superclass.name %>(name)
<% elsif task.needs_configuration? %>
    : <%= task.superclass.name %>(name, TaskCore::PreOperational)
<% else %>
    : <%= task.superclass.name %>(name, state)
<% end %>
    <% task.self_properties.each do |prop| %>
    , _<%= prop.name %>("<%= prop.name %>", "<%= prop.doc %>")<% end %>
    <% task.self_ports.each do |port| %>
    , _<%= port.name %>("<%= port.name %>")<% end %>
    <% task.new_methods.each do |meth| %>
    // If you get the following error:
    //   <%= meth.method_name %> is not a member of <%= task.name %>
    // it means that you did not implement the "<%= meth.method_name %>" method
    // in <%= task.name %>. Please update tasks/<%= task.basename %>.*
    // See
    //   templates/tasks/<%= task.basename %>.hpp and
    //   templates/tasks/<%= task.basename %>.cpp
    , _<%= meth.name %>("<%= meth.name %>", &<%= task.name %>Base::<%= meth.method_name %>, this)<% end %>
    <% task.new_commands.each do |cmd| %>
    // If you get one of the following errors:
    //   <%= cmd.work_method_name %> is not a member of <%= task.name %>
    //   <%= cmd.completion_method_name %> is not a member of <%= task.name %>
    // it means that you did not implement one of those two methods in in
    // <%= task.name %>. Please update tasks/<%= task.basename %>.*
    // See
    //   templates/tasks/<%= task.basename %>.hpp and
    //   templates/tasks/<%= task.basename %>.cpp
    , _<%= cmd.name %>("<%= cmd.name %>", &<%= task.name %>Base::<%= cmd.work_method_name %>, &<%= task.name %>Base::<%= cmd.completion_method_name %>, this)<% end %>

    <% if task.superclass.name == "RTT::TaskContext" %>
    , _getModelName("getModelName", &<%= task.name %>Base::getModelName, this)
    <% end %>
{
    <% task.self_properties.each do |prop|
        if prop.default_value %>
        _<%= prop.name %>.set(<%= prop.cxx_default_value %>);
        <% end %>
    properties()->addProperty( &_<%= prop.name %> );<% end %>
    <% (task.self_ports - task.event_ports).each do |port| %>
    ports()->addPort( &_<%= port.name %>, "<%= port.doc %>" );<% end %>
    <% (task.event_ports & task.self_ports).each do |port| %>
    ports()->addEventPort( &_<%= port.name %>, std::string("<%= port.doc %>") );<% end %>
    <% (task.new_methods + task.new_commands).each do |callable| 
	argument_setup = callable.arguments.
	    map { |n, _, d| ", \"#{n}\", \"#{d}\"" }.
	    join("")
	kind = callable.class.name.gsub(/^.*::/, '')
    %>
    <%= kind.downcase %>s()->add<%= kind %>( &_<%= callable.name %>, "<%= callable.doc %>"<%= argument_setup %>);<% end %>
    <% if task.superclass.name == "RTT::TaskContext" %>
    methods()->addMethod( &_getModelName, "getModelName()");
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
bool <%= task.basename %>Base::activate()
{
    StateExporter exporter(*this, _state);
    return <%= superclass.name %>::activate();
}
void <%= task.basename %>Base::recovered()
{
    StateExporter exporter(*this, _state);
    return <%= superclass.name %>::recovered();
}
void <%= task.basename %>Base::warning()
{
    StateExporter exporter(*this, _state);
    return <%= superclass.name %>::warning();
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
bool <%= task.basename %>Base::resetError()
{
    StateExporter exporter(*this, _state);
    return <%= superclass.name %>::resetError();
}
void <%= task.basename %>Base::fatal()
{ return fatal(FATAL_ERROR); }
void <%= task.basename %>Base::error()
{ return error(RUNTIME_ERROR); }
<% end %>

