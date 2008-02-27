#ifndef <%= task.name.upcase %>_TASK_HPP
#define <%= task.name.upcase %>_TASK_HPP

#include "tasks/<%= task.name %>Base.hpp"

namespace <%= component.name %> {
    class <%= task.name %> : public <%= task.name %>Base
    {
    };
}

#endif

