#ifndef <%= name.upcase %>_TOOLKIT_PLUGIN_HPP
#define <%= name.upcase %>_TOOLKIT_PLUGIN_HPP

#include <rtt/Toolkit.hpp>

namespace <%= name %> {
    class ToolkitPlugin
	: public RTT::ToolkitPlugin
    {
	bool loadTypes();
	std::string getName();

	bool loadOperators();
	bool loadConstructors();
    };

    extern ToolkitPlugin Toolkit;
}

#endif

