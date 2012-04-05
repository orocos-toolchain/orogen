/* Generated from orogen/lib/orogen/templates/typekit/typelib/TypelibMarshallerBase.hpp */

#ifndef OROGEN_TYPELIB_MARSHALLER_BASE_HPP
#define OROGEN_TYPELIB_MARSHALLER_BASE_HPP

#include <rtt/types/TypeTransporter.hpp>
#include <typelib/value_ops.hh>

namespace orogen_transports
{
    static const int TYPELIB_MARSHALLER_ID = 42;

    class TypelibMarshallerBase : public RTT::types::TypeTransporter
    {
        bool m_plain;
        Typelib::Registry const* registry;
        Typelib::Type const*  type_def;
        Typelib::MemoryLayout layout;
        std::string m_typename_typelib;
        std::string m_typename_orocos;

    public:
        struct Handle;

        TypelibMarshallerBase(bool plain_type,
                std::string const& typelib_typename,
                std::string const& orocos_typename,
                Typelib::Registry const& registry);

        /** Returns the typelib registry that defines this type
         */
        Typelib::Registry const& getRegistry() const;

        /** Returns true if the underlying orocos type can be manipulated by
         * typelib directly and false otherwise
         */
        bool isPlainTypelibType() const;

        /** Returns the Typelib-friendly type name for this marshaller. I.e. the
         * type name for the pointer returned by getTypelibData.
         *
         * I.e. if the type is opaque, it is the name of the intermediate type.
         * Otherwise, it is the type name itself.
         */
        char const* getMarshallingType() const;

        /** \overload
         */
        void setTypelibSample(Handle* data, Typelib::Value typelib_data, bool refresh_orocos = true);

        /** Updates the sample handler by using a data sample that Typelib
         * understands. +typelib_data+ must be pointing to an object whose type
         * is the one returned by getMarshallingType.
         *
         * The handle will then point directly to the given data sample, i.e.
         * calls to \c read might modify that sample directly.
         *
         * The ownership of \c typelib_data is retained by the caller. I.e. it
         * has to delete it.
         */
        virtual void setTypelibSample(Handle* data, uint8_t* typelib_data, bool refresh_orocos = true) = 0;

        /** Updates the sample handler by using a data sample that Typelib
         * understands. +typelib_data+ must be pointing to an object whose type
         * is the one returned by getMarshallingType.
         *
         * The handle will then point directly to the given data sample, i.e.
         * calls to \c read might modify that sample directly.
         *
         * The ownership of \c typelib_data is retained by the caller. I.e. it
         * has to delete it.
         */
        virtual void setOrocosSample(Handle* data, void* sample, bool refresh_typelib = true) = 0;

        /** Updates the value of data->orocos_sample based on the data in
         * data->typelib_sample
         */
        virtual void refreshOrocosSample(Handle* data) = 0;

        /** Updates the value of data->typelib_sample based on the data in
         * data->orocos_sample
         */
        virtual void refreshTypelibSample(Handle* handle) = 0;

        /** Returns a type-pruned pointer to an object that Typelib understands.
         * This object is of the type returned by getMarshallingType
         *
         * If this handle has ownership of the sample, it is kept (i.e. not
         * passed along). Note that the sample would need to have been
         * either explicitly set or explicitly refreshed for this value to be
         * non-NULL.
         */
        uint8_t* getTypelibSample(Handle* sample);

        /** Returns a type-pruned pointer to an object that Typelib understands.
         * This object is of the type returned by getMarshallingType
         *
         * If this handle has ownership of the sample, it is kept (i.e. not
         * passed along). Note that the sample would need to have been
         * either explicitly set or explicitly refreshed for this value to be
         * non-NULL.
         */
        uint8_t* getOrocosSample(Handle* sample);

        /** Returns a type-pruned pointer to the C++ object, and passes
         * ownership along
         */
        virtual uint8_t* releaseOrocosSample(Handle* sample) = 0;

        /** Creates a sample handler, which is an opaque type used to
         * read/write/marshal data. Unlike createHandle(), the handle returned
         * by this method already contains a preallocated data sample. It can
         * therefore be used as-is for readPort.
         */
        virtual Handle* createSample() = 0;

        /** Creates a sample handler, which is an opaque type used to
         * read/write/marshal data. Unlike createSample(), the handle returned
         * by this method does not contain any preallocated data sample. Using
         * it in readPort will cause memory allocation.
         */
        Handle* createHandle();

        /** Destroys the orocos-related data sample that is stored in \c data.
         *
         * For non-opaque types, this is the same than destroyOrocosSample and
         * destroyData. It is different only for opaque types.
         */
        virtual void deleteOrocosSample(Handle* data) = 0;

        /** Destroys the typelib-related data sample that is stored in \c data.
         *
         * For non-opaque types, this is the same than destroyTypelibSample and
         * destroyData. It is different only for opaque types.
         */
        virtual void deleteTypelibSample(Handle* data) = 0;

        /** Destroys the sample handle and the data it owns.
         *
         * What is actually deleted depends on how the handle has been used. For
         * instance, if setTypelibSample has been used, then the sample will not
         * be deleted.
         */
        void deleteHandle(Handle* data);

        /** Returns a data source that can be used to modify the Orocos sample
         * stored in this handle
         */
        virtual RTT::base::DataSourceBase::shared_ptr getDataSource(Handle* handle) = 0;

        /** Writes the data from from the handle into the data source
         */
        virtual void writeDataSource(RTT::base::DataSourceBase& source, Handle const* handle) = 0;

        /** Reads the data from a data source a handle
         */
        virtual bool readDataSource(RTT::base::DataSourceBase& source, Handle* handle) = 0;

        /** Returns the marshalled size for the given data sample
         */
        size_t getMarshallingSize(Handle const* sample) const;
        /** Marshals the given sample directly on to a file
         */
        void marshal(int fd, Handle* sample);
        /** Marshals the given sample directly on to a C++ stream
         */
        void marshal(std::ostream& stream, Handle* sample);
        /** Marshals the given sample into a memory buffer
         */
        void marshal(std::vector<uint8_t>& buffer, Handle* sample);
        /** Marshals the given sample into a memory buffer
         */
        int marshal(void* buffer, int buffer_size, Handle* sample);
        /** Update the sample in +sample+ from the marshalled data in +buffer+
         */
        virtual void unmarshal(std::vector<uint8_t>& buffer, Handle* sample);
        /** Update the sample in +sample+ from the marshalled data in +buffer+
         */
        virtual void unmarshal(void const* buffer, int buffer_size, Handle* sample);

        virtual RTT::base::ChannelElementBase::shared_ptr createStream(RTT::base::PortInterface* port, const RTT::ConnPolicy& policy, bool is_sender) const
        { return RTT::base::ChannelElementBase::shared_ptr(); }
    };

    /** Given a typelib value, returns the corresponding opaque value
     *
     * The caller is the owner of the memory. He is responsible to cast it to
     * the right type, and delete it. The string is the type name of the opaque
     * value.
     *
     * Note that for values that are typelib-compatible, a copy of the input is
     * returned so that the ownership constraint outlined above is met.
     *
     * It relies on using the RTT type system to find the right
     * TypelibMarshallerBase object.
     */
    void* getOpaqueValue(std::string const& expected_type, Typelib::Value value);

    /** Returns a TypelibMarshallerBase object that can handle the provided type
     */
    orogen_transports::TypelibMarshallerBase* getMarshallerFor(std::string const& type);
}

#endif

