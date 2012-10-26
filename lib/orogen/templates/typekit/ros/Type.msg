<% type.each_field do |field_name, field_type| %>
<%= ROS::MsgGeneration.message_name_for(typekit.name, field_type) %> <%= field_name %>
<% end %>

