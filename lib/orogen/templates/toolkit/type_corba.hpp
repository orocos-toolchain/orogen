<%
    type_name = type.cxx_name
    corba_type_name = type.corba_name
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
<%= result = ""
	    type.code_to_corba(toolkit, result, "", " " * 8)
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
<%= result = ""
	    type.code_from_corba(toolkit, result, "", " " * 8)
	    result 
	%>
    }
    <% end %>
};

