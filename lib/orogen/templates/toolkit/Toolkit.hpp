#ifndef <%= component.name.upcase %>_TOOLKIT_HPP
#define <%= component.name.upcase %>_TOOLKIT_HPP

#include <rtt/ToolkitPlugin.hpp>

namespace Typelib {
    class Registry;
}

namespace orogen_toolkits {
    class <%= component.name %>ToolkitPlugin
        : public RTT::ToolkitPlugin
    {
        Typelib::Registry* m_registry;

    public:
        <%= component.name %>ToolkitPlugin();
        ~<%= component.name %>ToolkitPlugin();
        bool loadTypes();
        bool loadOperators();
        bool loadConstructors();
        std::string getName();
    };

    extern <%= component.name %>ToolkitPlugin <%= component.name %>Toolkit;
}

#endif


