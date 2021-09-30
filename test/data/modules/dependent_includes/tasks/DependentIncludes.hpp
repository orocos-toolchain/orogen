#pragma once

#include "dependent_includes/DependentIncludesBase.hpp"

namespace dependent_includes {
    class DependentIncludes : public DependentIncludesBase
    {
	friend class DependentIncludesBase;

    public:
        DependentIncludes(std::string const& name = "dependent_includes::DependentIncludes", TaskCore::TaskState initial_state = Stopped);
        DependentIncludes(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

    };
}

