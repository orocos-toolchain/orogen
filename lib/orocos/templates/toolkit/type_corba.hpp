template<>
struct AnyConversion< <%= name %>::<%= type.basename %> >
{
    typedef <%= name %>::Corba::<%= type.basename %> CorbaType;
    typedef <%= name %>::<%= type.basename %> BaseType;

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

    static BaseType get(const CorbaType* _value) {
	BaseType   result;
	CorbaType const& value = *_value;
	int i;
<%= result = ""
	    type.code_from_corba(result, "", " " * 8)
	    result 
	%>
	return result;
    }

    static bool update(const CORBA::Any& any, BaseType& value) {
	CorbaType* result;
	if ( any >>= result ) {
	    value = get(result);
	    return true;
	}
	return false;
    }

    static CORBA::Any_ptr createAny( const BaseType& t ) {
	CORBA::Any_ptr ret = new CORBA::Any();
	*ret <<= toAny( t );
	return ret;
    }
};

