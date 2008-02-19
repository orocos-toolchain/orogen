
struct <%= type.basename %>TypeInfo :
  public RTT::TemplateTypeInfo<<%= type.basename %>>
{
  <%= type.basename %>TypeInfo()
    : RTT::TemplateTypeInfo<<%= type.basename %>>("<%= type.basename %>")

  bool decomposeTypeImpl(const <%= type.basename %>& value, PropertyBag& target_bag) const {
    target_bag.setType("<%= type.basename %>")
<%= result = ""
      type.to_orocos_decomposition(result, "") 
      result
    %> 
    return true;
  }

  bool composeTypeImpl(const PropertyBag& bag, <%= type.basename %>& out) {
    <%= type.to_orocos_composition %>
  }
};

