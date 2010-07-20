#ifndef <%= component.name.upcase %>_TOOLKIT_HPP
#define <%= component.name.upcase %>_TOOLKIT_HPP

#include <rtt/types/TypekitPlugin.hpp>

namespace Typelib {
    class Registry;
}

namespace orogen_typekits {
    class <%= component.name %>TypekitPlugin
        : public RTT::types::TypekitPlugin
    {
        Typelib::Registry* m_registry;

    public:
        <%= component.name %>TypekitPlugin();
        ~<%= component.name %>TypekitPlugin();
        bool loadTypes();
        bool loadOperators();
        bool loadConstructors();
        std::string getName();
    };

    extern <%= component.name %>TypekitPlugin <%= component.name %>Typekit;
}

#endif


