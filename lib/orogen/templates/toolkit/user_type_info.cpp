
<% begin %>
    std::ostream& operator << (std::ostream& io, <%= type.basename %> const& data) {
        <%= intermediate_type.cxx_name %> temp;
        to_intermediate(temp, data);
        io << temp;
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
            <%= intermediate_type.cxx_name %> temp;
            to_intermediate(temp, value);
            return <%= intermediate_type.cxx_name %>TypeInfo::doDecompose(temp, target_bag);
        }
	bool decomposeTypeImpl(const <%= type.basename %>& value, RTT::PropertyBag& target_bag) const {
	    target_bag.setType("<%= type.full_name %>");
            return doDecompose(value, target_bag);
	}

	static bool doCompose(const RTT::PropertyBag& bag, <%= type.basename %>& out) {
            <%= intermediate_type.cxx_name %> temp;
            if (! <%= intermediate_type.cxx_name %>TypeInfo::doCompose(bag, temp))
                return false;

            from_intermediate(out, temp);
            return true;
        }

	bool composeTypeImpl(const RTT::PropertyBag& bag, <%= type.basename %>& out) {
            return doCompose(bag, out);
	}
    };
<% rescue TypeError => e
    raise TypeError, "cannot include #{type.name} in the toolkit: #{e}"
   end %>

