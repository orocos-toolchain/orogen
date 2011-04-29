#ifndef OROGEN_TYPELIB_MARSHALLER_HPP
#define OROGEN_TYPELIB_MARSHALLER_HPP

#include <rtt/typelib/TypelibMarshallerBase.hpp>
#include <rtt/typelib/TypelibMarshallerHandle.hpp>
#include <rtt/internal/DataSources.hpp>

namespace orogen_transports
{
    template<typename T>
    struct TypelibMarshaller : public orogen_transports::TypelibMarshallerBase
    {
        typedef TypelibMarshallerBase::Handle Handle;

        TypelibMarshaller(std::string const& orocos_name, Typelib::Registry const& registry)
            : orogen_transports::TypelibMarshallerBase(true, orocos_name, orocos_name, registry) { }

        TypelibMarshallerBase::Handle* createSample() { return new Handle(this, new T); }
        void deleteOrocosSample(Handle* data)  { deleteSamples(data); }
        void deleteTypelibSample(Handle* data) { deleteSamples(data); }
        void deleteSamples(Handle* data)
        {
            delete reinterpret_cast<T*>(data->orocos_sample);
            data->reset();
        }

        void setTypelibSample(Handle* handle, uint8_t* data, bool refresh_orocos = true)
        {
            handle->orocos_sample = handle->typelib_sample = reinterpret_cast<uint8_t*>(data);
            handle->owns_orocos = handle->owns_typelib = false;
        }

        void setOrocosSample(Handle* handle, void* data, bool refresh_typelib = true)
        {
            handle->orocos_sample = handle->typelib_sample = reinterpret_cast<uint8_t*>(data);
            handle->owns_orocos = handle->owns_typelib = false;
        }

        uint8_t* releaseOrocosSample(Handle* handle)
        {
            if (!handle->orocos_sample)
                return 0;

            if (handle->owns_orocos)
            {
                return reinterpret_cast<uint8_t*>(new T(*reinterpret_cast<T*>(handle->orocos_sample)));
            }
            else
            {
                handle->owns_orocos = false;
                handle->owns_typelib = false;
                return handle->orocos_sample;
            }
        }

        void refreshTypelibSample(Handle* handle)
        {
            // Nothing to do
        }

        void refreshOrocosSample(Handle* handle)
        {
            // Nothing to do
        }

        RTT::base::DataSourceBase::shared_ptr getDataSource(Handle* handle)
        {
            return new RTT::internal::ReferenceDataSource<T>(*reinterpret_cast<T*>(handle->orocos_sample));
        }

        bool readDataSource(RTT::base::DataSourceBase& source_base, Handle* handle)
        {
            RTT::internal::DataSource<T>& source = dynamic_cast<RTT::internal::DataSource<T>&>(source_base);
            if (source.evaluate())
            {
                T& data = *reinterpret_cast<T*>(handle->orocos_sample);
                data = dynamic_cast<RTT::internal::DataSource<T>&>(source_base).get();
                return true;
            }
            return false;
        }
        void writeDataSource(RTT::base::DataSourceBase& source, Handle const* handle)
        {
            T const& data = *reinterpret_cast<T const*>(handle->orocos_sample);
            dynamic_cast<RTT::internal::AssignableDataSource<T>&>(source).set(data);
        }
    };
}

#endif

