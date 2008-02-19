#ifndef <%= toolkit_name %>_TOOLKIT_PLUGIN_HPP
#define <%= toolkit_name %>_TOOLKIT_PLUGIN_HPP

class <%= toolkit_name %>ToolkitPlugin
    : public RTT::ToolkitPlugin
{
    bool loadTypes();
};

extern <%= toolkit_name %> ToolkitPlugin <%= toolkit_name >Toolkit;

#endif

