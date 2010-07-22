<%= into.arg_type %> <%= typekit.name %>::to_intermediate(<%= from.arg_type %> real_type)
{
    return *real_type;
}
bool <%= typekit.name %>::from_intermediate(<%= from.ref_type %> real_type, <%= into.cxx_name %>* intermediate)
{
    real_type.reset(intermediate);
    return true; // the smart pointer took ownership of the pointer
}
