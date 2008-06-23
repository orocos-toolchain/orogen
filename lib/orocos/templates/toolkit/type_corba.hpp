<%
    type_name = type.full_name('::', true)
    corba_type_name = "#{type.namespace('::')}Corba::#{type.basename}"
%>

template<>
struct AnyConversion< <%= type_name %> > 
    : public details::conversion_base< AnyConversion< <%= type_name %> >, <%= type_name %>, <%= corba_type_name %>* >
    , public details::PODHelper< <%= type_name %>, <%= corba_type_name %> >
{
    typedef <%= corba_type_name %> CorbaType;
    typedef <%= type_name %> BaseType;

    <% if toolkit.blob_threshold && type.size > toolkit.blob_threshold %>
    static CorbaType* toAny(const BaseType& value) {
	return new CorbaType(sizeof(BaseType), sizeof(BaseType), (CORBA::Octet*)&value);
    }
    <% else %>
    static CorbaType* toAny(const BaseType& value) {
	CorbaType* _result = new CorbaType();
	CorbaType&  result = *_result;
	int i;
<%= result = ""
	    type.code_to_corba(result, "", " " * 8)
	    result 
	%>
	return _result;
    }
    <% end %>

    <% if toolkit.blob_threshold && type.size > toolkit.blob_threshold %>
    static void get(const CorbaType* _value, BaseType& ret) {
	ret = *((BaseType*)_value->get_buffer());
    }
    <% else %>
    static void get(const CorbaType* _value, BaseType& result) {
	CorbaType const& value = *_value;
	int i;
<%= result = ""
	    type.code_from_corba(result, "", " " * 8)
	    result 
	%>
    }
    <% end %>
};

