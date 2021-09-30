#include "DependentIncludes.hpp"
#include <cross_consumer/TaskStates.hpp>

using namespace dependent_includes;

DependentIncludes::DependentIncludes(std::string const& name, TaskCore::TaskState initial_state)
    : DependentIncludesBase(name, initial_state)
{
}

DependentIncludes::DependentIncludes(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : DependentIncludesBase(name, initial_state)
{
}