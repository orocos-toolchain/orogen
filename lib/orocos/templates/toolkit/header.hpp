#ifndef <%= name.upcase %>_TOOLKIT_PLUGIN_HPP
#define <%= name.upcase %>_TOOLKIT_PLUGIN_HPP

#include <rtt/Toolkit.hpp>

namespace <%= name %> {
    static const int ORO_UNTYPED_PROTOCOL_ID = 42;
    class BufferGetterBase;

    class ToolkitPlugin
	: public RTT::ToolkitPlugin
    {
    public:
	bool loadTypes();
	std::string getName();

	bool loadOperators();
	bool loadConstructors();
    };

    extern ToolkitPlugin Toolkit;
}

#endif

