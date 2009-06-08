
<% begin %>
    std::ostream& operator << (std::ostream& io, <%= type.cxx_basename %> const& data) {
<%= result = ""
        type.to_ostream(toolkit, result, "", " " * 4)
        result %>
        return io;
    }

    std::istream& operator >> (std::istream& io, <%= type.cxx_basename %>& data) {
        return io;
    }

    struct <%= type.method_name(false) %>TypeInfo :
	public RTT::TemplateTypeInfo<<%= type.cxx_name %>, true>
    {
        <%= type.method_name(false) %>TypeInfo()
	    : RTT::TemplateTypeInfo<<%= type.cxx_name %>, true>("<%= type.full_name %>") {}

	static bool doDecompose(const <%= type.cxx_name %>& value, RTT::PropertyBag& target_bag) {
<%= result = ""
	    type.to_orocos_decomposition(toolkit, result, "", " " * 12) 
	    result
	    %> 
            return true;
        }

	bool decomposeTypeImpl(const <%= type.cxx_name %>& value, RTT::PropertyBag& target_bag) const {
	    target_bag.setType("<%= type.full_name %>");
	    return doDecompose(value, target_bag);
	}

	static bool doCompose(const RTT::PropertyBag& bag, <%= type.cxx_name %>& out) {
<%= result = ""
		type.from_orocos_decomposition(toolkit, result, "", " " * 12)
		result %>
            return true;
        }
	bool composeTypeImpl(const RTT::PropertyBag& bag, <%= type.cxx_name %>& out) const {
	    return doCompose(bag, out);
	}
    };
<% rescue TypeError => e
    raise TypeError, "cannot include #{type.name} in the toolkit: #{e}", e.backtrace
   end %>

