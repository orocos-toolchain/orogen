#ifndef OROGEN_MQ_TYPELIB_MARSHALLER_HPP
#define OROGEN_MQ_TYPELIB_MARSHALLER_HPP

#include <rtt/transports/mqueue/MQTemplateProtocol.hpp>

namespace orogen_transports {
    class TypelibMarshallerBase;

    class MQTypelibMarshallerBase
    {
        orogen_transports::TypelibMarshallerBase* marshaller;

    public:
        MQTypelibMarshallerBase(orogen_transports::TypelibMarshallerBase* marshaller);
        virtual ~MQTypelibMarshallerBase();
        void* createCookie() const;
        void deleteCookie(void* cookie) const;
        std::pair<void const*,int> fillBlob( RTT::base::DataSourceBase::shared_ptr source, void* blob, int size, void* cookie) const;
        bool updateFromBlob(const void* blob, int size, RTT::base::DataSourceBase::shared_ptr target, void* cookie) const;
        unsigned int getSampleSize( RTT::base::DataSourceBase::shared_ptr sample, void* cookie) const;
    };

    template<typename T>
    class MQTypelibMarshaller : public RTT::mqueue::MQTemplateProtocol<T>, public MQTypelibMarshallerBase {
    public:
        MQTypelibMarshaller(orogen_transports::TypelibMarshallerBase* marshaller)
            : MQTypelibMarshallerBase(marshaller) {}

        void* createCookie() const
        { return MQTypelibMarshallerBase::createCookie(); }
        void deleteCookie(void* cookie) const
        { return MQTypelibMarshallerBase::deleteCookie(cookie); }
        std::pair<void const*,int> fillBlob( RTT::base::DataSourceBase::shared_ptr source, void* blob, int size, void* cookie) const
        { return MQTypelibMarshallerBase::fillBlob(source, blob, size, cookie); }
        bool updateFromBlob(const void* blob, int size, RTT::base::DataSourceBase::shared_ptr target, void* cookie) const
        { return MQTypelibMarshallerBase::updateFromBlob(blob, size, target, cookie); }
        unsigned int getSampleSize( RTT::base::DataSourceBase::shared_ptr sample, void* cookie) const
        { return MQTypelibMarshallerBase::getSampleSize(sample, cookie); }
    };
}

#endif
