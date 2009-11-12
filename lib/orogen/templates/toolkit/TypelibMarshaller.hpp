#ifndef OROGEN_TYPELIB_MARSHALLER_HPP
#define OROGEN_TYPELIB_MARSHALLER_HPP

#include <rtt/TypeTransporter.hpp>

namespace orogen_transports
{
    static const int TYPELIB_MARSHALLER_ID = 42;
    class TypelibMarshallerBase : public RTT::detail::TypeTransporter
    {
    public:
        virtual char const* getMarshallingType() const = 0;
        virtual void marshal(std::vector<uint8_t>& buffer, RTT::DataSourceBase::shared_ptr data) const = 0;

        virtual void* createBlob(RTT::DataSourceBase::shared_ptr source) const
        { return NULL; }
        virtual void* reuseBlob(void* blob, RTT::DataSourceBase::shared_ptr source) const
        { return NULL; }
        virtual bool updateBlob(const void* blob, RTT::DataSourceBase::shared_ptr target) const
        { return NULL; }

        RTT::DataSourceBase* proxy(void* data ) const { return 0; }
        void* server(RTT::DataSourceBase::shared_ptr source, bool assignable, void* arg) const { return 0; }
        void* method(RTT::DataSourceBase::shared_ptr source, RTT::MethodC* orig, void* arg) const { return 0; }
        RTT::DataSourceBase* dataProxy( RTT::PortInterface* data ) const { return 0; }
        RTT::DataSourceBase* dataProxy( void* data ) const { return 0; }
        void* dataServer( RTT::DataSourceBase::shared_ptr source, void* arg) const { return 0; }
        RTT::BufferBase* bufferProxy( RTT::PortInterface* data ) const { return 0; }
        RTT::BufferBase* bufferProxy( void* data ) const { return 0; }
        void* bufferServer( RTT::BufferBase::shared_ptr source, void* arg) const { return 0; }
        RTT::DataSourceBase* narrowDataSource(RTT::DataSourceBase* dsb) { return 0; }
        RTT::DataSourceBase* narrowAssignableDataSource(RTT::DataSourceBase* dsb) { return 0; }
    };
}

#endif

