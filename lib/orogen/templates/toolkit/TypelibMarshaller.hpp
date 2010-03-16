#ifndef OROGEN_TYPELIB_MARSHALLER_HPP
#define OROGEN_TYPELIB_MARSHALLER_HPP

#include <rtt/TypeTransporter.hpp>
#include <rtt/PortInterface.hpp>

namespace orogen_transports
{
    static const int TYPELIB_MARSHALLER_ID = 42;
    class TypelibMarshallerBase : public RTT::detail::TypeTransporter
    {
    public:
        struct Handle;

        virtual char const* getMarshallingType() const = 0;

        virtual Handle* createSample() = 0;
        virtual void destroySample(Handle* data) = 0;

        virtual bool readPort(RTT::InputPortInterface& port, Handle* sample) = 0;
        // virtual bool writePort(OutputPortBase& port, uint8_t* sample) = 0;

        virtual size_t getMarshallingSize(Handle const* sample) const = 0;
        virtual void marshal(int fd, Handle* sample) const = 0;
        virtual void marshal(std::ostream& stream, Handle* sample) const = 0;
        virtual void marshal(std::vector<uint8_t>& buffer, Handle* sample) const = 0;

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

