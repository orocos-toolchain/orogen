#ifndef <%= component.name.upcase %>_TOOLKIT_PLUGIN_HPP
#define <%= component.name.upcase %>_TOOLKIT_PLUGIN_HPP

#include <rtt/Toolkit.hpp>

namespace Typelib {
    class Registry;
}

namespace <%= component.name %> {
    static const int ORO_UNTYPED_PROTOCOL_ID = 42;
    class BufferGetterBase;

    class ToolkitPlugin
	: public RTT::ToolkitPlugin
    {
        Typelib::Registry* m_registry;
    public:

        ToolkitPlugin();
        ~ToolkitPlugin();

	bool loadTypes();
	std::string getName();

	bool loadOperators();
	bool loadConstructors();
    };

    extern ToolkitPlugin Toolkit;
}

#endif

