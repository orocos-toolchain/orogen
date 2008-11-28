
<% begin %>
    std::ostream& operator << (std::ostream& io, <%= type.basename %> const& data) {
<%= result = ""
        type.to_ostream(toolkit, result, "", " " * 4)
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

	static bool doDecompose(const <%= type.basename %>& value, RTT::PropertyBag& target_bag) {
        <%= result = ""
	    type.to_orocos_decomposition(toolkit, result, "", " " * 12) 
	    result
	    %> 
            return true;
        }

	bool decomposeTypeImpl(const <%= type.basename %>& value, RTT::PropertyBag& target_bag) const {
	    target_bag.setType("<%= type.full_name %>");
	    return doDecompose(value, target_bag);
	}

	static bool doCompose(const RTT::PropertyBag& bag, <%= type.basename %>& out) {
	    <%= result = ""
		type.from_orocos_decomposition(toolkit, result, "", " " * 12)
		result %>
            return true;
        }
	bool composeTypeImpl(const RTT::PropertyBag& bag, <%= type.basename %>& out) const {
	    return doCompose(bag, out);
	}
    };
<% rescue TypeError => e
    raise TypeError, "cannot include #{type.name} in the toolkit: #{e}"
   end %>

