#ifndef OPAQUE_TYPELIB_MARRSHALLER_HPP
#define OPAQUE_TYPELIB_MARRSHALLER_HPP

#include <rtt/typelib/TypelibMarshallerBase.hpp>
#include <rtt/typelib/TypelibMarshallerHandle.hpp>
#include <rtt/internal/DataSources.hpp>

namespace orogen_transports {

template<typename TypelibType, typename OpaqueType>
struct OpaqueTypelibMarshallerBase : public orogen_transports::TypelibMarshallerBase
{
    typedef orogen_transports::TypelibMarshallerBase::Handle MarshallingHandle;
    typedef OpaqueType  opaque_t;
    typedef TypelibType typelib_t;

    OpaqueTypelibMarshallerBase(std::string const& typelib_name, std::string const& opaque_name, Typelib::Registry const& registry)
        : orogen_transports::TypelibMarshallerBase(false, typelib_name, opaque_name, registry) { }

    RTT::base::DataSourceBase::shared_ptr getDataSource(Handle* handle)
    {
        ensureOrocosSamplePresent(handle);
        return new RTT::internal::ReferenceDataSource< opaque_t >(*reinterpret_cast< opaque_t* >(handle->orocos_sample));
    }

    void deleteOrocosSample(MarshallingHandle* data)
    { delete reinterpret_cast<opaque_t*>(data->orocos_sample); }
    void deleteTypelibSample(MarshallingHandle* data)
    { delete reinterpret_cast<typelib_t*>(data->typelib_sample); }

    uint8_t* releaseOrocosSample(Handle* handle)
    {
        if (!handle->orocos_sample)
            return 0;

        if (handle->owns_orocos)
        {
            return reinterpret_cast<uint8_t*>(new opaque_t(*reinterpret_cast<opaque_t*>(handle->orocos_sample)));
        }
        else
        {
            handle->owns_orocos = false;
            uint8_t* result = handle->orocos_sample;
            handle->orocos_sample = 0;
            return result;
        }
    }

    void ensureOrocosSamplePresent(MarshallingHandle* handle)
    {
        if (!handle->orocos_sample)
        {
            handle->orocos_sample = reinterpret_cast<uint8_t*>(new opaque_t);
            handle->owns_orocos  = true;
        }
    }

    void ensureTypelibSamplePresent(MarshallingHandle* handle)
    {
        if (!handle->typelib_sample)
        {
            handle->typelib_sample = reinterpret_cast<uint8_t*>(new typelib_t);
            handle->owns_typelib  = true;
        }
    }

    void setOrocosSample(MarshallingHandle* handle, void* data, bool refresh_typelib = true)
    {
        if (handle->owns_orocos)
            deleteOrocosSample(handle);
        handle->orocos_sample = reinterpret_cast<uint8_t*>(data);
        handle->owns_orocos = false;

        if (refresh_typelib)
            refreshTypelibSample(handle);
    }

    bool readDataSource(RTT::base::DataSourceBase& source_base, MarshallingHandle* handle)
    {
        RTT::internal::DataSource<opaque_t>& source = dynamic_cast<RTT::internal::DataSource<opaque_t>&>(source_base);
        if (source.evaluate())
        {
            ensureOrocosSamplePresent(handle);
            opaque_t& opaque_sample =
                *reinterpret_cast<opaque_t*>(handle->orocos_sample);
            opaque_sample = source.value();
            refreshTypelibSample(handle);
            return true;
        }
        return false;
    }
    void writeDataSource(RTT::base::DataSourceBase& source, MarshallingHandle const* handle)
    {
        opaque_t const& data = *reinterpret_cast<opaque_t const*>(handle->orocos_sample);
        dynamic_cast<RTT::internal::AssignableDataSource<opaque_t>&>(source).set(data);
    }

    void unmarshal(void const* buffer, int size, Handle* handle)
    {
        ensureTypelibSamplePresent(handle);
        TypelibMarshallerBase::unmarshal(buffer, size, handle);
        refreshOrocosSample(handle);
    }

    void unmarshal(std::vector<uint8_t>& buffer, Handle* handle)
    {
        ensureTypelibSamplePresent(handle);
        TypelibMarshallerBase::unmarshal(buffer, handle);
        refreshOrocosSample(handle);
    }
};

template<typename TypelibType, typename OpaqueType, int needs_copy>
class OpaqueTypelibMarshaller;

template<typename TypelibType, typename OpaqueType>
struct OpaqueTypelibMarshaller<TypelibType, OpaqueType, 0> : public OpaqueTypelibMarshallerBase<TypelibType, OpaqueType>
{
    typedef orogen_transports::TypelibMarshallerBase::Handle MarshallingHandle;
    typedef OpaqueType  opaque_t;
    typedef TypelibType typelib_t;

    OpaqueTypelibMarshaller(std::string const& typelib_name, std::string const& opaque_name, Typelib::Registry const& registry)
        : OpaqueTypelibMarshallerBase<TypelibType, OpaqueType>(typelib_name, opaque_name, registry) {}

    MarshallingHandle* createSample()
    {
        return new MarshallingHandle(this, (typelib_t*)0, new opaque_t);
    }

    void refreshTypelibSample(MarshallingHandle* handle)
    {
        this->ensureOrocosSamplePresent(handle);
        opaque_t& opaque_sample =
            *reinterpret_cast<opaque_t*>(handle->orocos_sample);

        if (handle->typelib_sample && handle->owns_typelib)
            this->deleteTypelibSample(handle);

        handle->typelib_sample = const_cast<uint8_t*>(
                reinterpret_cast<uint8_t const*>(
                    &orogen_typekits::toIntermediate(opaque_sample) ));
        handle->owns_typelib = false;
    }

    void refreshOrocosSample(MarshallingHandle* handle)
    { 
        this->ensureOrocosSamplePresent(handle);

        opaque_t& opaque_sample =
            *reinterpret_cast<opaque_t*>(handle->orocos_sample);
        typelib_t& typelib_sample =
            *reinterpret_cast<typelib_t*>(handle->typelib_sample);
        handle->owns_typelib =
            !orogen_typekits::fromIntermediate(opaque_sample, &typelib_sample);
    }

    void setTypelibSample(MarshallingHandle* handle, uint8_t* typelib_data, bool refresh_orocos = true)
    {
        // The orocos sample might take ownership of the typelib sample
        // To avoid too complicated ownership logic for the users of this class,
        // we do a copy first.
        if (!handle->typelib_sample || !handle->owns_typelib)
        {
            handle->typelib_sample = reinterpret_cast<uint8_t*>(new typelib_t);
            handle->owns_typelib = true;
        }
        typelib_t& new_value =
            *reinterpret_cast<typelib_t*>(typelib_data);
        typelib_t& intermediate_sample =
            *reinterpret_cast<typelib_t*>(handle->typelib_sample);
        intermediate_sample = new_value;

        if (refresh_orocos)
            refreshOrocosSample(handle);
    }
};

template<typename TypelibType, typename OpaqueType>
struct OpaqueTypelibMarshaller<TypelibType, OpaqueType, 1> : public OpaqueTypelibMarshallerBase<TypelibType, OpaqueType>
{
    typedef orogen_transports::TypelibMarshallerBase::Handle MarshallingHandle;
    typedef OpaqueType  opaque_t;
    typedef TypelibType typelib_t;

    OpaqueTypelibMarshaller(std::string const& typelib_name, std::string const& opaque_name, Typelib::Registry const& registry)
        : OpaqueTypelibMarshallerBase<TypelibType, OpaqueType>(typelib_name, opaque_name, registry) {}

    MarshallingHandle* createSample()
    {
        return new MarshallingHandle(this, new typelib_t, new opaque_t);
    }

    void refreshTypelibSample(MarshallingHandle* handle)
    {
        this->ensureOrocosSamplePresent(handle);
        opaque_t& opaque_sample =
            *reinterpret_cast<opaque_t*>(handle->orocos_sample);

        this->ensureTypelibSamplePresent(handle);
        typelib_t& intermediate_sample =
            *reinterpret_cast<typelib_t*>(handle->typelib_sample);
        orogen_typekits::toIntermediate(intermediate_sample, opaque_sample);
    }

    void refreshOrocosSample(MarshallingHandle* handle)
    { 
        this->ensureOrocosSamplePresent(handle);

        opaque_t& opaque_sample =
            *reinterpret_cast<opaque_t*>(handle->orocos_sample);
        typelib_t& typelib_sample =
            *reinterpret_cast<typelib_t*>(handle->typelib_sample);
        orogen_typekits::fromIntermediate(opaque_sample, typelib_sample);
    }

    void setTypelibSample(MarshallingHandle* handle, uint8_t* typelib_data, bool refresh_orocos = true)
    {
        if (handle->owns_typelib)
            this->deleteTypelibSample(handle);
        handle->typelib_sample = typelib_data;
        handle->owns_typelib   = false;

        if (refresh_orocos)
            refreshOrocosSample(handle);
    }
};

}

#endif

