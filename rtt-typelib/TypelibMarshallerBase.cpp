#include "TypelibMarshallerBase.hpp"
#include "TypelibMarshallerHandle.hpp"

#include <rtt/types/TypeInfoRepository.hpp>

using namespace orogen_transports;

typedef TypelibMarshallerBase::Handle Handle;

TypelibMarshallerBase::TypelibMarshallerBase(bool plain,
        std::string const& typelib_typename,
        std::string const& orocos_typename,
        Typelib::Registry const& registry)
    : m_plain(plain)
    , registry(&registry)
    , type_def(registry.get(typelib_typename))
    , m_typename_typelib(typelib_typename)
    , m_typename_orocos(orocos_typename)
{
    if (!type_def)
        throw std::runtime_error(typelib_typename + " is not present in the typekit registry");

    layout    = Typelib::layout_of(*type_def, false, false);
}

Typelib::Registry const& TypelibMarshallerBase::getRegistry() const
{ return *registry; }

bool TypelibMarshallerBase::isPlainTypelibType() const
{ return m_plain; }

Handle* TypelibMarshallerBase::createHandle() { return new Handle(this); }
void TypelibMarshallerBase::deleteHandle(Handle* handle) { delete handle; }
uint8_t* TypelibMarshallerBase::getTypelibSample(Handle* handle)
{
    return handle->typelib_sample;
}
uint8_t* TypelibMarshallerBase::getOrocosSample(Handle* handle)
{
    return handle->orocos_sample;
}
char const* TypelibMarshallerBase::getMarshallingType() const
{ return m_typename_typelib.c_str(); }
size_t TypelibMarshallerBase::getMarshallingSize(Handle const* handle) const
{ return Typelib::getDumpSize(handle->typelib_sample, layout); }
void TypelibMarshallerBase::marshal(int fd, Handle* handle)
{ Typelib::dump(handle->typelib_sample, fd, layout); }
void TypelibMarshallerBase::marshal(std::ostream& stream, Handle* handle)
{ Typelib::dump(handle->typelib_sample, stream, layout); }
void TypelibMarshallerBase::marshal(std::vector<uint8_t>& buffer, Handle* handle)
{ Typelib::dump(handle->typelib_sample, buffer, layout); }
int TypelibMarshallerBase::marshal(void* buffer, int buffer_size, Handle* handle)
{ return Typelib::dump(handle->typelib_sample, static_cast<uint8_t*>(buffer), buffer_size, layout); }
void TypelibMarshallerBase::unmarshal(void const* buffer, int buffer_size, Handle* handle)
{
    Typelib::load(handle->typelib_sample, *type_def, static_cast<uint8_t const*>(buffer), buffer_size, layout);
    refreshOrocosSample(handle);
}
void TypelibMarshallerBase::unmarshal(std::vector<uint8_t>& buffer, Handle* handle)
{
    Typelib::load(handle->typelib_sample, *type_def, buffer, layout);
    refreshOrocosSample(handle);
}
void TypelibMarshallerBase::setTypelibSample(Handle* data, Typelib::Value typelib_data, bool refresh_orocos)
{ return setTypelibSample(data, reinterpret_cast<uint8_t*>(typelib_data.getData()), refresh_orocos); }

orogen_transports::TypelibMarshallerBase* orogen_transports::getMarshallerFor(std::string const& type)
{
    RTT::types::TypeInfoRepository::shared_ptr type_registry =
        RTT::types::TypeInfoRepository::Instance();
    RTT::types::TypeInfo* ti = type_registry->type(type);
    if (!ti)
    {
	// Try harder. Some base types don't have a
	// typelib-normalized name, so we should look
	// for the type without the leading slash
	ti = type_registry->type(type.substr(1));
	if (!ti)
	    throw std::runtime_error("type " + type + " is not registered in the RTT type system");
    }

    if (!ti->hasProtocol(orogen_transports::TYPELIB_MARSHALLER_ID))
        throw std::runtime_error("type " + type + " is registered in the RTT type system, but does not have a typelib transport");

    orogen_transports::TypelibMarshallerBase* typelib_marshaller =
        dynamic_cast<orogen_transports::TypelibMarshallerBase*>(ti->getProtocol(orogen_transports::TYPELIB_MARSHALLER_ID));
    if (!typelib_marshaller)
        throw std::runtime_error("the transport object registered as typelib transport for type " + type + " is not a TypelibMarshallerBase");

    return typelib_marshaller;
}

void* orogen_transports::getOpaqueValue(std::string const& expected_type, Typelib::Value value)
{
    orogen_transports::TypelibMarshallerBase* typelib_marshaller =
        getMarshallerFor(expected_type);

    orogen_transports::TypelibMarshallerBase::Handle* handle =
        typelib_marshaller->createHandle();
    typelib_marshaller->setTypelibSample(handle, reinterpret_cast<uint8_t*>(value.getData()), true);
    void* opaque_sample = typelib_marshaller->releaseOrocosSample(handle);
    typelib_marshaller->deleteHandle(handle);
    return opaque_sample;
}

