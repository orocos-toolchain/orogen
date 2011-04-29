#include "TypelibMarshallerBase.hpp"
#include "MQTypelibMarshaller.hpp"

using namespace orogen_transports;

typedef TypelibMarshallerBase::Handle TypelibHandle;

MQTypelibMarshallerBase::MQTypelibMarshallerBase(orogen_transports::TypelibMarshallerBase* marshaller)
    : marshaller(marshaller) {}

MQTypelibMarshallerBase::~MQTypelibMarshallerBase()
{
    delete marshaller;
}

void* MQTypelibMarshallerBase::createCookie() const
{
    return marshaller->createHandle();
}

void MQTypelibMarshallerBase::deleteCookie(void* cookie) const
{
    marshaller->deleteHandle(static_cast<TypelibHandle*>(cookie));
}

std::pair<void const*,int> MQTypelibMarshallerBase::fillBlob( RTT::base::DataSourceBase::shared_ptr source, void* blob, int size, void* cookie) const
{
    TypelibHandle* handle = static_cast<TypelibHandle*>(cookie);
    marshaller->setOrocosSample(handle, const_cast<void*>(source->getRawConstPointer()));
    int marshalled_size = marshaller->marshal(blob, size, handle);
    if (marshalled_size == 0)
        return std::make_pair(static_cast<void const*>(0), 0);
    else
        return std::make_pair(static_cast<void const*>(blob), marshalled_size);
}

bool MQTypelibMarshallerBase::updateFromBlob(const void* blob, int size, RTT::base::DataSourceBase::shared_ptr target, void* cookie) const
{
    TypelibHandle* handle = static_cast<TypelibHandle*>(cookie);
    marshaller->setOrocosSample(handle, target->getRawPointer(), false);
    marshaller->unmarshal(blob, size, handle);
    return true;
}

unsigned int MQTypelibMarshallerBase::getSampleSize( RTT::base::DataSourceBase::shared_ptr sample, void* cookie) const
{
    TypelibHandle* handle = static_cast<TypelibHandle*>(cookie);
    marshaller->setOrocosSample(handle, const_cast<void*>(sample->getRawConstPointer()));
    return marshaller->getMarshallingSize(handle);
}

