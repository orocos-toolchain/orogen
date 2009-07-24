<%= into.cxx_name %> const& <%= component.name %>::to_intermediate(<%= from.cxx_name %> const& real_type)
{
    return *real_type;
}
bool <%= component.name %>::from_intermediate(<%= from.cxx_name %>& real_type, <%= into.cxx_name %>* intermediate)
{
    real_type.reset(intermediate);
    return true; // the smart pointer took ownership of the pointer
}

