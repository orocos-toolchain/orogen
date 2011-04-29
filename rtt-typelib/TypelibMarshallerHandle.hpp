#ifndef TYPELIB_MARSHALLER_HANDLE_HPP
#define TYPELIB_MARSHALLER_HANDLE_HPP

namespace orogen_transports
{
    struct TypelibMarshallerBase::Handle
    {
        /** The TypelibMarshallerBase that created us
         */
        TypelibMarshallerBase* marshaller;
        /** Type-pruned pointer of the sample that Typelib can understand. It may be
         * the same as orocos_sample
         */
        uint8_t* typelib_sample;
        /** If true, the handle owns the memory addressed by \c typelib_sample, and
         * will therefore destroy it when it gets deleted.
         */
        bool owns_typelib;
        /** Type-pruned pointer of the sample that Orocos can understand. It may be
         * the same as typelib_sample
         */
        uint8_t* orocos_sample;
        /** If true, the handle owns the memory addressed by \c orocos_sample, and
         * will therefore destroy it when it gets deleted.
         */
        bool owns_orocos;

        void reset()
        {
            typelib_sample = 0;
            orocos_sample  = 0;
        }

        Handle(TypelibMarshallerBase* marshaller)
            : marshaller(marshaller)
            , typelib_sample(0), owns_typelib(true)
            , orocos_sample(0), owns_orocos(true) {}

        template<typename Type>
        Handle(TypelibMarshallerBase* marshaller, Type* data)
            : marshaller(marshaller)
            , typelib_sample(reinterpret_cast<uint8_t*>(data))
            , owns_typelib(true)
            , orocos_sample(typelib_sample)
            , owns_orocos(true) {}

        template<typename TypelibType, typename OrocosType>
        Handle(TypelibMarshallerBase* marshaller, TypelibType* typelib_data, OrocosType* orocos_data)
            : marshaller(marshaller)
            , typelib_sample(reinterpret_cast<uint8_t*>(typelib_data))
            , owns_typelib(true)
            , orocos_sample(reinterpret_cast<uint8_t*>(orocos_data))
            , owns_orocos(true) {}

        ~Handle()
        {
            if (owns_typelib && typelib_sample)
                marshaller->deleteTypelibSample(this);
            if (owns_orocos && orocos_sample)
                marshaller->deleteOrocosSample(this);
        }
    };
}
#endif

