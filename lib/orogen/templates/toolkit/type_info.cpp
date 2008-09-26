
<% begin %>
    std::ostream& operator << (std::ostream& io, <%= type.basename %> const& data) {
<%= result = ""
        type.to_ostream(result, "", " " * 4)
        result %>
        return io;
    }

    std::istream& operator >> (std::istream& io, <%= type.basename %>& data) {
        return io;
    }

    struct <%= type.basename %>TypeInfo :
	public RTT::TemplateTypeInfo<<%= type.basename %>, true>
    {
        <%= type.basename %>TypeInfo()
	    : RTT::TemplateTypeInfo<<%= type.basename %>, true>("<%= type.full_name %>") {}

	bool decomposeTypeImpl(const <%= type.basename %>& value, RTT::PropertyBag& target_bag) const {
	    target_bag.setType("<%= type.full_name %>");
<%= result = ""
	    type.to_orocos_decomposition(result, "", " " * 12) 
	    result
	    %> 
	    return true;
	}

	bool composeTypeImpl(const RTT::PropertyBag& bag, <%= type.basename %>& out) {
	    <%= type.to_orocos_composition %>
	    return false;
	}
    };
<% rescue TypeError => e
    raise TypeError, "cannot include #{type.name} in the toolkit: #{e}"
   end %>
