#define OROCOS_TARGET gnulinux
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/typekit/RealTimeTypekit.hpp>
#include "simple/typekit/Types.hpp"
#include "simple/typekit/Plugin.hpp"
#include <string.h>

#ifdef WITH_CORBA
#include <omniORB4/CORBA.h>
#include <rtt/transports/corba/CorbaLib.hpp>
#include <rtt/transports/corba/CorbaTypeTransporter.hpp>
#include "transports/corba/simpleTypesC.h"
#include "transports/corba/TransportPlugin.hpp"
#endif

#ifdef WITH_TYPELIB
#include "transports/typelib/TransportPlugin.hpp"
#include <rtt/typelib/TypelibMarshallerBase.hpp>
#endif

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
using std::cerr;
using std::endl;

TypeInfo* assert_typeinfo_is_available(std::string const& name)
{
    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    TypeInfo* type = ti->type(name);
    if (! type)
    {
	cerr << "cannot find " << name << " in the type info repository" << endl;
	throw std::runtime_error("failed assertion");
    }
    return type;
}

#ifdef WITH_CORBA
template<typename T>
bool generic_corba_test(T const& testValue, RTT::types::TypeInfo const& ti)
{
    boost::intrusive_ptr< ValueDataSource<T> > source(new ValueDataSource<T>(testValue));
    source->ref();

    RTT::corba::CorbaTypeTransporter* transport =
        dynamic_cast<RTT::corba::CorbaTypeTransporter*>(ti.getProtocol(ORO_CORBA_PROTOCOL_ID));

    cerr << "- testing CORBA marshalling/unmarshalling ..." << endl;
    {
        CORBA::Any* result = transport->createAny(source);

        ValueDataSource<T>* reader = new ValueDataSource<T>();
        reader->ref();
        transport->updateFromAny(result, reader);

        T result_value = reader->get();
        if (!(result_value == testValue))
        {
            cerr << "discrepancy between original and unmarshalled values" << endl;
            return false;
        }
    }
    cerr << "  done" << endl;
    return true;
}
#endif

#ifdef WITH_TYPELIB
template<typename T>
bool generic_typelib_test(T const& testValue, TypeInfo const& ti)
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
    T unmarshalled =
        reinterpret_cast<T const&>(*transport->getTypelibSample(handle));
    transport->deleteHandle(handle);

    if (!(unmarshalled == testValue))
    {
        cerr << "unmarshalled Typelib data does not match original data" << endl;
        return false;
    }
    return true;
}
#endif


template<typename T>
bool generic_type_handling_test(std::string const& name, T const& testValue, TypeInfo const& ti)
{
    cerr << "- testing composition/decomposition is disabled" << endl;
    //cerr << "- testing decomposition to XML formats" << endl;
    //ValueDataSource<T>* source
    //    = new ValueDataSource<T>(testValue);
    //source->ref();

    //PropertyBag bag;
    //if (!typeDecomposition(source, bag))
    //{
    //    cerr << "  decomposition failed" << endl;
    //    return false;
    //}
    //cerr << "  done decomposition" << endl;

    //std::vector<std::string> fields = bag.list();
    //cerr << "  " << fields.size() << " element(s) in the bag:";
    //for (int i = 0; i < fields.size(); ++i)
    //    cerr << " " << fields[i];
    //cerr << endl;

    //// Now, marshal it to the standard Orocos format, reload it and compare
    //// the result
    //PropertyMarshaller cpf_output(name + ".cpf");
    //cpf_output.serialize(bag);
    //cpf_output.flush();
    //cerr << "  done CPF serialization" << endl;

    //cerr << "- testing unmarshalling from XML" << endl;
    //PropertyBag input_bag;
    //PropertyDemarshaller cpf_input(name + ".cpf");
    //cpf_input.deserialize(input_bag);

    //{ ValueDataSource<T>* reader = new ValueDataSource<T>();
    //    reader->ref();
    //    Property<PropertyBag> bag("", "", input_bag);
    //    if (!ti.composeType(bag.getDataSource(), reader))
    //    {
    //        cerr << "cannot recompose type" << endl;
    //        return false;
    //    }

    //    T value = reader->get();
    //    if (!(value == testValue))
    //    {
    //        cerr << "error, expected and actual values differ" << endl;
    //        return false;
    //    }
    //}
    //cerr << "  done" << endl;

#ifdef WITH_CORBA
    if (!generic_corba_test(testValue, ti))
        return false;
#endif
#ifdef WITH_TYPELIB
    if (!generic_typelib_test(testValue, ti))
        return false;
#endif
    return true;
}

bool test_base_types()
{
    cerr << "Test handling of BaseTypes" << endl;
    TypeInfo* type = assert_typeinfo_is_available("/Test/BaseTypes");

    // Create a structure of type Test::BaseTypes, inject it into Orocos and check
    // that it is able to generate a XML representation of it
    Test::BaseTypes testValue;
    testValue.v0 = true;
    testValue.v1 = 'a';
    testValue.v5 = -100000;
    testValue.v6 = 3000000000UL;
    testValue.e   = Test::VALUE_20;
    for (int i = 0; i < 20; ++i)
	testValue.a[i] = 'a' + i;

    if (!generic_type_handling_test("basic", testValue, *type))
        return false;
    return true;
}

bool test_handling_of_invalid_enum_values()
{
    cerr << "- testing composition/decomposition is disabled" << endl;
    //cerr << "test_handling_of_invalid_enum_values" << endl;
    //TypeInfo* type = assert_typeinfo_is_available("/Test/BaseTypes");
    //Test::BaseTypes testValue;
    //memset(&testValue, 0, sizeof(testValue));
    //testValue.e   = (Test::BASIC_ENUM)999;

    //boost::intrusive_ptr< ValueDataSource<Test::BaseTypes> > source(new ValueDataSource<Test::BaseTypes>(testValue));
    //source->ref();

    //// Decomposition should fail
    //cerr << "- testing that decomposition is forbidden" << endl;
    //PropertyBag bag;
    //if (typeDecomposition(source, bag))
    //{
    //    cerr << "  FAILED" << endl;
    //    return false;
    //}

    // TODO: test that CORBA fails as well
    return true;
}

bool test_64bit_handling()
{
    cerr << "- testing composition/decomposition is disabled" << endl;
    //cerr << "Testing 64bit handling" << endl;
    TypeInfo* type = assert_typeinfo_is_available("/Test/Test64BitHandling");

    Test::Test64BitHandling testValue;
    memset(&testValue, 0, sizeof(testValue));
    testValue.base.v0 = true;
    testValue.base.v1 = 'a';
    testValue.base.v5 = -100000;
    testValue.base.v6 = 3000000000UL;
    testValue.base.e   = Test::VALUE_20;
    for (int i = 0; i < 20; ++i)
        testValue.base.a[i] = 'a' + i;
    testValue.ll  = -(1LL << 40);
    testValue.ull = -(1LL << 40);

    //boost::intrusive_ptr< ValueDataSource<Test::Test64BitHandling> > source(new ValueDataSource<Test::Test64BitHandling>(testValue));
    //source->ref();

    //// Decomposition should fail
    //cerr << "- testing that decomposition is forbidden" << endl;
    //PropertyBag bag;
    //if (typeDecomposition(source, bag))
    //{
    //    cerr << "  FAILED" << endl;
    //    return false;
    //}

    // But CORBA and typelib marshalling should work just fine
#ifdef WITH_CORBA
    if (!generic_corba_test(testValue, *type))
        return false;
#endif
#ifdef WITH_TYPELIB
    if (!generic_typelib_test(testValue, *type))
        return false;
#endif
    return true;
}

bool test_simple_vector()
{
    cerr << "Testing simple structure with vector<>" << endl;
    TypeInfo* type = assert_typeinfo_is_available("/Test/SimpleVector");

    Test::SimpleVector testValue;
    testValue.field = 10;
    for (int i = 0; i < 20; ++i)
        testValue.data.push_back('a' + i);

    return generic_type_handling_test("simple_vector", testValue, *type);
}

bool test_complex_vector()
{
    cerr << "Testing complex structure-in-vector<>" << endl;
    TypeInfo* type = assert_typeinfo_is_available("/Test/ComplexVector");

    Test::ComplexVector testValue;
    testValue.field = 10;
    testValue.data.resize(20);
    for (int i = 0; i < 20; ++i)
    {
        Test::SimpleVector& el = testValue.data[i];
        el.field = i;
        for (int j = 0; j < i + 5; ++j)
            el.data.push_back(i * 20 + j);
    }

    return generic_type_handling_test("complex_vector", testValue, *type);
}

bool test_complex_array()
{
    cerr << "Testing complex structure-in-array" << endl;
    TypeInfo* type = assert_typeinfo_is_available("/Test/ComplexArray");

    Test::ComplexArray testValue;
    testValue.field = 10;
    for (int i = 0; i < 10; ++i)
    {
        Test::SimpleVector& el = testValue.data[i];
        el.field = i;
        for (int j = 0; j < i + 5; ++j)
            el.data.push_back(i * 20 + j);
    }

    return generic_type_handling_test("complex_array", testValue, *type);
}




int ORO_main(int argc, char** argv)
{
    log().setLogLevel( Logger::Debug );
    RTT::types::TypekitRepository::Import( new RTT::types::RealTimeTypekitPlugin );
    RTT::types::TypekitRepository::Import( new orogen_typekits::simpleTypekitPlugin );
#ifdef WITH_CORBA
    RTT::types::TypekitRepository::Import( new orogen_typekits::simpleCorbaTransportPlugin );
#endif
#ifdef WITH_TYPELIB
    RTT::types::TypekitRepository::Import( new orogen_typekits::simpleTypelibTransportPlugin );
#endif

    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();

    if (!test_base_types())
        return 1;

    cerr << endl;
    if (!test_handling_of_invalid_enum_values())
        return 1;

    cerr << endl;
    if (!test_64bit_handling())
        return 1;

    cerr << endl;
    if (!test_simple_vector())
        return 1;

    cerr << endl;
    if (!test_complex_vector())
        return 1;

    cerr << endl;
    if (!test_complex_array())
        return 1;

    return 0;
}

