#define OROCOS_TARGET gnulinux
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/typekit/RealTimeTypekit.hpp>
#include <opaque/typekit/Types.hpp>
#include <opaque/typekit/Plugin.hpp>

#ifdef WITH_CORBA
#include <omniORB4/CORBA.h>
#include <rtt/transports/corba/CorbaLib.hpp>
#include <rtt/transports/corba/CorbaTypeTransporter.hpp>
#include "build/.orogen/typekit/transports/corba/opaqueTypesC.h"
#include ".orogen/typekit/transports/corba/TransportPlugin.hpp"
#endif

#ifdef WITH_TYPELIB
#include "transports/typelib/TransportPlugin.hpp"
#include <rtt/typelib/TypelibMarshallerBase.hpp>
#endif

#include <opaque/typekit/OpaqueConvertions.hpp>

#include <rtt/os/main.h>
#include <rtt/types/Types.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>
#include <rtt/types/PropertyDecomposition.hpp>
#include <rtt/marsh/PropertyMarshaller.hpp>
#include <rtt/marsh/PropertyDemarshaller.hpp>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <stdexcept>

using namespace RTT;
using namespace RTT::types;
using namespace RTT::internal;
using namespace RTT::marsh;
using namespace orogen_typekits;
using std::cerr;
using std::endl;

namespace TestOpaque {
    inline std::ostream& operator << (std::ostream& io, Position_m const& data) {
        io << "{ .timestamp = " << data.timestamp << ", .p.x" << data.p.x << ", .p.y" << data.p.y << "}";
        return io;
    }
}
namespace std {
    extern std::ostream& operator << (std::ostream& io, std::vector<float> const& data);
    extern std::ostream& operator << (std::ostream& io, std::vector<int> const& data);
}
template<typename T>
T identity(T const& value) { return value; }

#ifdef WITH_TYPELIB
template<typename OrocosType, typename TypelibType>
void fromIntermediate(OrocosType& orocos_value, TypelibType& typelib_value)
{
    ValueDataSource<OrocosType>* data_source
        = new ValueDataSource<OrocosType>();
    data_source->ref();

    TypeInfo const* type = data_source->getTypeInfo();
    orogen_transports::TypelibMarshallerBase* transport =
        dynamic_cast<orogen_transports::TypelibMarshallerBase*>(type->getProtocol(orogen_transports::TYPELIB_MARSHALLER_ID));

    orogen_transports::TypelibMarshallerBase::Handle* handle =
        transport->createSample();
    transport->setTypelibSample(handle, reinterpret_cast<uint8_t*>(&typelib_value));
    transport->writeDataSource(*data_source, handle);
    orocos_value = data_source->get();

    transport->deleteHandle(handle);
    data_source->deref();
}

template<typename T, typename Getter>
bool generic_typelib_test(T const& testValue, TypeInfo const& ti, Getter get_test_value)
{
    cerr << "- testing Typelib marshalling/unmarshalling ..." << endl;
    ConstantDataSource<T>* source
        = new ConstantDataSource<T>(testValue);
    source->ref();

    orogen_transports::TypelibMarshallerBase* transport =
        dynamic_cast<orogen_transports::TypelibMarshallerBase*>(ti.getProtocol(orogen_transports::TYPELIB_MARSHALLER_ID));

    std::vector<uint8_t> buffer;
    orogen_transports::TypelibMarshallerBase::Handle* handle =
        transport->createSample();
    transport->readDataSource(*source, handle);
    transport->marshal(buffer, handle);
    transport->deleteHandle(handle);
    source->deref();

    handle = transport->createSample();
    transport->unmarshal(buffer, handle);
    ValueDataSource<T> unmarshalled;
    transport->writeDataSource(unmarshalled, handle);
    transport->deleteHandle(handle);

    if (!(get_test_value(unmarshalled.get()) == get_test_value(testValue)))
    {
        cerr << "unmarshalled Typelib data does not match original data" << endl;
        return false;
    }
    return true;
}
#endif

/* This is a generic test of type handling.
 * - it marshals the type into a XML file. This file can then be compared
 *   with an expected content by the Ruby test case.
 * - it does the same with a CPF file
 * - it unmarshals the CPF file and checks that both values are equal
 * - if the test is done with CORBA, it also converts to/from any and compares
 *   the two values.
 */
template<typename T, typename Getter>
bool generic_type_handling_test(std::string const& name, T const& testValue, TypeInfo const& ti,
        Getter get_test_value)
{
    ConstantDataSource<T>* source
        = new ConstantDataSource<T>(testValue);
    source->ref();

    std::cerr << "disabled XML decomposition/recomposition test" << std::endl;

    //PropertyBag bag;
    //ti.decomposeType(source, bag);

    //// First, save it into XML. The Ruby test case will compare that to an
    //// expected XML document
    //std::ofstream xml_file((name + ".xml").c_str());
    //XMLMarshaller<std::ostream> xml_output(xml_file);
    //xml_output.serialize(bag);

    //// Now, marshal it to the standard Orocos format, reload it and compare
    //// the result
    //PropertyMarshaller cpf_output(name + ".cpf");
    //cpf_output.serialize(bag);
    //cpf_output.flush();

    //PropertyBag input_bag;
    //PropertyDemarshaller cpf_input(name + ".cpf");
    //cpf_input.deserialize(input_bag);

    //cerr << "Testing PropertyBag composition" << endl;
    //{ ValueDataSource<T>* reader = new ValueDataSource<T>();
    //    reader->ref();
    //    Property<PropertyBag> bag("", "", input_bag);
    //    if (!ti.composeType(bag.getDataSource(), reader))
    //    {
    //        cerr << "cannot recompose type" << endl;
    //        return false;
    //    }

    //    T value = reader->get();
    //    if (!(get_test_value(value) == get_test_value(testValue)))
    //    {
    //        cerr << "error. Expected\n\n" << get_test_value(testValue) <<
    //            "\n\nand got\n\n" << get_test_value(value) << endl;
    //    }
    //    reader->deref();
    //}

#ifdef WITH_CORBA
    std::cerr << "Testing CORBA marshalling/unmarshalling ..." << std::endl;
    { RTT::corba::CorbaTypeTransporter* transport =
            dynamic_cast<RTT::corba::CorbaTypeTransporter*>(ti.getProtocol(ORO_CORBA_PROTOCOL_ID));

        CORBA::Any* result = transport->createAny(source);

        ValueDataSource<T>* reader = new ValueDataSource<T>();
        reader->ref();
        transport->updateFromAny(result, reader);

        T value = reader->get();
        if (!(get_test_value(value) == get_test_value(testValue)))
        {
            cerr << "error in CORBA marshalling/demarshalling" << endl;
            return false;
        }
        delete result;
        reader->deref();
    }
#endif

#ifdef WITH_TYPELIB
    if (!generic_typelib_test(testValue, ti, get_test_value))
        return false;
#endif

    source->deref();
    return true;
}

bool test_plain_opaque()
{
    cerr << "\n======== Testing plain opaque handling ========" << endl;

    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    TypeInfo* type = ti->type("/NotOrogenCompatible/Point2D");
    if (! type)
    {
	cerr << "cannot find /NotOrogenCompatible/Point2D in the type info repository" << endl;
	return false;
    }

    std::cerr << "Testing the initialization of the types from an intermediate..." << std::endl;
    {
        TestOpaque::Point2D testValue = { 100, 20, 30 };
        NotOrogenCompatible::Point2D p(0, 0);
        fromIntermediate(p, testValue);

        if (testValue.x != p.x() || testValue.y != p.y())
        {
            cerr << "error in Typelib marshalling" << endl;
            return false;
        }
    }


    NotOrogenCompatible::Point2D testValue(10, 20);
    if (!generic_type_handling_test("opaque", testValue, *type, &::identity<NotOrogenCompatible::Point2D> ))
        return false;

    return true;
}

bool test_composed_opaque()
{
    cerr << "\n======== Testing opaque field in struct ========" << endl;

    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    TypeInfo* type = ti->type("/TestOpaque/Position");
    if (! type)
    {
	cerr << "cannot find /TestOpaque/Position in the type info repository" << endl;
	return 1;
    }

    TestOpaque::Position testValue;
    testValue.timestamp = 10;
    testValue.p.x() = 20;
    testValue.p.y() = 30;

    generic_type_handling_test("composed_opaque", testValue, *type, &::identity<TestOpaque::Position> );

    std::cerr << "Testing the initialization of the types from an intermediate..." << std::endl;
    {
        TestOpaque::Position_m testValue = { 100, { 20, 30 } };
        TestOpaque::Position p;
        memset(&p, 0, sizeof(p));
        fromIntermediate(p, testValue);

        if (testValue.timestamp != p.timestamp || testValue.p.x != p.p.x() || testValue.p.y != p.p.y())
        {
            cerr << "error in Typelib marshalling" << endl;
            return false;
        }
    }

    return true;
}

template<typename T>
T get_ptr_content(boost::shared_ptr<T> const& left)
{ return *left; }

template<typename T>
T get_ro_ptr_content(RTT::extras::ReadOnlyPointer<T> const& left)
{ return *left; }

bool test_shared_pointer()
{
    cerr << "\n======== Testing shared pointer ========" << endl;

    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    TypeInfo* type = ti->type("/boost/shared_ptr</std/vector</float>>");
    if (! type)
    {
	cerr << "cannot find /boost/shared_ptr</std/vector</float>> in the type info repository" << endl;
	return false;
    }

    boost::shared_ptr< std::vector<float> > testValue( new std::vector<float> );
    std::vector<float>& data = *testValue;

    for (int i = 0; i < 10; ++i)
        data.push_back(i);

    generic_type_handling_test("shared_ptr__opaque_type", testValue, *type, &get_ptr_content< std::vector<float> >);

    std::cerr << "Testing the initialization of the types from an intermediate..." << std::endl;
    {
        boost::shared_ptr< std::vector<float> > result;
        fromIntermediate(result, *testValue);

        for (int i = 0; i < data.size(); ++i)
            if ((*result)[i] != (*testValue)[i])
            {
                cerr << "error in type initialization from an intermediate" << endl;
                return false;
            }
    }

    return true;
}

bool test_shared_ptr_shortcut()
{
    cerr << "\n======== Testing shared pointer (from #shared_ptr) ========" << endl;

    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    TypeInfo* type = ti->type("/boost/shared_ptr</std/vector</int32_t>>");
    if (! type)
    {
	cerr << "cannot find /boost/shared_ptr</std/vector</int32_t>> in the type info repository" << endl;
	return false;
    }

    boost::shared_ptr< std::vector<int> > testValue( new std::vector<int> );
    std::vector<int>& data = *testValue;

    for (int i = 0; i < 10; ++i)
        data.push_back(i);


    generic_type_handling_test("shared_ptr__shared_ptr", testValue, *type, &get_ptr_content< std::vector<int> >);

    std::cerr << "Testing the initialization of the types from an intermediate..." << std::endl;
    {
        boost::shared_ptr< std::vector<int> > result;
        fromIntermediate(result, data);

        for (int i = 0; i < data.size(); ++i)
            if ((*result)[i] != (*testValue)[i])
            {
                cerr << "error in type initialization from an intermediate" << endl;
                return false;
            }
    }

    return true;
}

bool test_ro_ptr()
{
    cerr << "\n======== Testing ReadOnlyPointer (from #ro_ptr) ========" << endl;

    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    TypeInfo* type = ti->type("/RTT/extras/ReadOnlyPointer</std/vector</int32_t>>");
    if (! type)
    {
	cerr << "cannot find /RTT/extras/ReadOnlyPointer</std/vector</int32_t>> in the type info repository" << endl;
	return false;
    }

    std::vector<int>* data = new std::vector<int>();
    for (int i = 0; i < 10; ++i)
        data->push_back(i);

    RTT::extras::ReadOnlyPointer< std::vector<int> > testValue(data);


    generic_type_handling_test("readonlypointer", testValue, *type, &get_ro_ptr_content< std::vector<int> >);

    std::cerr << "Testing the initialization of the types from an intermediate..." << std::endl;
    {
        RTT::extras::ReadOnlyPointer< std::vector<int> > result;
        fromIntermediate(result, *data);

        for (int i = 0; i < data->size(); ++i)
            if ((*result)[i] != (*testValue)[i])
            {
                cerr << "error in type initialization from an intermediate" << endl;
                return false;
            }
    }

    return true;
}


int ORO_main(int argc, char** argv)
{
    log().setLogLevel( Logger::Debug );
    RTT::types::TypekitRepository::Import( new RTT::types::RealTimeTypekitPlugin );
    RTT::types::TypekitRepository::Import( new orogen_typekits::opaqueTypekitPlugin );
#ifdef WITH_CORBA
    RTT::types::TypekitRepository::Import( new orogen_typekits::opaqueCorbaTransportPlugin );
#endif
#ifdef WITH_TYPELIB
    RTT::types::TypekitRepository::Import( new orogen_typekits::opaqueTypelibTransportPlugin );
#endif

    if (!test_plain_opaque())
    {
        cerr << "plain_opaque failed" << endl;
        return 1;
    }
    if (!test_composed_opaque())
    {
        cerr << "composed_opaque failed" << endl;
        return 1;
    }
    if (!test_shared_pointer())
    {
        cerr << "shared_ptr failed" << endl;
        return 1;
    }
    if (!test_shared_ptr_shortcut())
    {
        cerr << "shared_ptr (from #shared_ptr) failed" << endl;
        return 1;
    }
    if (!test_ro_ptr())
    {
        cerr << "ReadOnlyPointer failed" << endl;
        return 1;
    }

    return 0;
}

