#ifndef <%= component.name.upcase %>_TOOLKIT_HPP
#define <%= component.name.upcase %>_TOOLKIT_HPP

#include <rtt/ToolkitPlugin.hpp>

namespace orogen_toolkits {
    class <%= component.name %>ToolkitPlugin
        : public RTT::ToolkitPlugin
    {
    public:
        bool loadTypes();
        bool loadOperators();
        bool loadConstructors();
        std::string getName();
    };

    extern <%= component.name %>ToolkitPlugin <%= component.name %>Toolkit;
}

#endif


