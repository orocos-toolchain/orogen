#include <rtt/os/main.h>
<% if component.toolkit %>#include "toolkit/<%= toolkit.name %>Toolkit.hpp"<% end %>

int ORO_main(int argc, char* argv[])
{
    <% if component.toolkit %>RTT::Toolkit::Import( <%= component.toolkit.name %>::Toolkit );<% end %>
    return 0;
}

