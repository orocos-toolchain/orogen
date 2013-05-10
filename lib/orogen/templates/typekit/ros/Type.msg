<% type.each_field do |field_name, field_type| %>
<%= ros_field_name(field_type) %> <%= field_name %>

<% end %>

