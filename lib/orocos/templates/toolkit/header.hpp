#ifndef <%= toolkit_name.upcase %>_TOOLKIT_PLUGIN_HPP
#define <%= toolkit_name.upcase %>_TOOLKIT_PLUGIN_HPP

#include <rtt/Toolkit.hpp>

namespace <%= toolkit_name %> {
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

