#ifndef OROCOS_<%= toolkit_name.upcase %>_CORBA_HPP
#define OROCOS_<%= toolkit_name.upcase %>_CORBA_HPP

namespace RTT {
    <% generated_types.each do |type| %>
<%= Orocos::Generation.render_template 'toolkit/type_corba.hpp', binding %>
    <% end %>
}

#endif

