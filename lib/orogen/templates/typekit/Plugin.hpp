/* Generated from orogen/lib/orogen/templates/typekit/Plugin.hpp */
#pragma once

#include <rtt/types/TypekitPlugin.hpp>

namespace Typelib {
    class Registry;
}

namespace orogen_typekits {
    class <%= typekit.name %>TypekitPlugin
        : public RTT::types::TypekitPlugin
    {
        Typelib::Registry* m_registry;

    public:
        <%= typekit.name %>TypekitPlugin();
        ~<%= typekit.name %>TypekitPlugin();
        bool loadTypes();
        bool loadOperators();
        bool loadConstructors();
        std::string getName();
    };

    extern <%= typekit.name %>TypekitPlugin <%= typekit.name %>Typekit;
}


