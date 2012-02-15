#ifndef RTT_TYPELIB_TRANSPORT_PLUGIN
#define RTT_TYPELIB_TRANSPORT_PLUGIN

#include <rtt/types/TransportPlugin.hpp>

namespace Typelib
{
    class Registry;
}

namespace orogen_transports
{
    class TypelibTransportPlugin
        : public RTT::types::TransportPlugin
    {
    protected:
        std::string m_basename;
        Typelib::Registry* m_registry;

    public:
        TypelibTransportPlugin(std::string const& basename);
        ~TypelibTransportPlugin();
        virtual std::string getTlbPath() const = 0;

        Typelib::Registry& getRegistry() const
        { return *m_registry; }

        bool loadRegistry();

        std::string getTransportName() const;
        std::string getTypekitName() const;
        std::string getName() const;
    };
}

#endif

