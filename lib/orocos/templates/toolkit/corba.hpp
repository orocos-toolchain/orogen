#ifndef OROCOS_<%= name.upcase %>_CORBA_HPP
#define OROCOS_<%= name.upcase %>_CORBA_HPP

#include "<%= name %>ToolkitC.h"
#include <rtt/corba/CorbaConversion.hpp>

namespace RTT {
    <% generated_types.each do |type| %>
<%= Orocos::Generation.render_template 'toolkit/type_corba.hpp', binding %>
    <% end %>
}

#endif

