<%
    type_name = type.cxx_name
    corba_type_name = intermediate_type.corba_name
%>

template<>
struct AnyConversion< <%= type_name %> > 
    : public details::conversion_base< AnyConversion< <%= type_name %> >, <%= type_name %>, <%= corba_type_name %>* >
    , public details::PODHelper< <%= type_name %>, <%= corba_type_name %> >
{
    typedef <%= corba_type_name %> CorbaType;
    typedef <%= type_name %> BaseType;
    typedef <%= intermediate_type.cxx_name %> IntermediateType;

    <% if toolkit.blob_threshold && type.size > toolkit.blob_threshold %>
    static CorbaType* toAny(const BaseType& value) {
        <% if opaque_specification(type).needs_copy? %>
        IntermediateType temp;
        <%= component.name %>::to_intermediate(temp, value);
        <% else %>
        IntermediateType const& temp = <%= component.name %>::to_intermediate(value);
        <% end %>
	return new CorbaType(sizeof(IntermediateType), sizeof(IntermediateType), (CORBA::Octet*)&temp);
    }
    <% else %>
    static CorbaType* toAny(const BaseType& value) {
        <% if opaque_specification(type).needs_copy? %>
        IntermediateType temp;
        <%= component.name %>::to_intermediate(temp, value);
        <% else %>
        IntermediateType const& temp = <%= component.name %>::to_intermediate(value);
        <% end %>
        return AnyConversion< <%= intermediate_type.cxx_name %> >::toAny(temp);
    }
    <% end %>

    <% if toolkit.blob_threshold && type.size > toolkit.blob_threshold %>
    static void get(const CorbaType* _value, BaseType& ret) {
        <% if opaque_specification(type).needs_copy? %>
        IntermediateType temp;
	temp = *((IntermediateType*)_value->get_buffer());
        <%= component.name %>::from_intermediate(ret, temp);
        <% else %>
        std::auto_ptr<IntermediateType> temp(new IntermediateType);
        *temp = *((IntermediateType*)_value->get_buffer());
        if (<%= component.name %>::from_intermediate(ret, temp.get()))
            temp.release();
        <% end %>
    }
    <% else %>
    static void get(const CorbaType* _value, BaseType& result) {
        <% if opaque_specification(type).needs_copy? %>
        IntermediateType temp;
        AnyConversion< <%= intermediate_type.cxx_name %> >::get(_value, temp);
        <%= component.name %>::from_intermediate(result, temp);
        <% else %>
        std::auto_ptr<IntermediateType> temp(new IntermediateType);
        AnyConversion< <%= intermediate_type.cxx_name %> >::get(_value, *temp);
        if (<%= component.name %>::from_intermediate(result, temp.get()))
            temp.release();
        <% end %>
    }
    <% end %>
};

