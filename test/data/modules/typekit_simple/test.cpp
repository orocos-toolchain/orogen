#define OROCOS_TARGET gnulinux
#include <rtt/Typekit.hpp>
#include "simpleTypekitTypes.hpp"
#include "simpleTypekit.hpp"
#include <string.h>

#ifdef WITH_CORBA
#include <omniORB4/CORBA.h>
#include <rtt/corba/CorbaLib.hpp>
#include "build/.orogen/typekit/simpleTypekitC.h"
#include "simpleTypekitCorba.hpp"
#endif

#include <rtt/os/main.h>
#include <rtt/Types.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/DataSources.hpp>
#include <rtt/marsh/XMLMarshaller.hpp>
#include <rtt/marsh/PropertyMarshaller.hpp>
#include <rtt/marsh/PropertyDemarshaller.hpp>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <stdexcept>

using namespace RTT;
using std::cerr;
using std::endl;

namespace Test {
    extern std::ostream& operator << (std::ostream& io, Test::BaseTypes const& data);
    extern std::ostream& operator << (std::ostream& io, Test::Test64BitHandling const& data);
    extern std::ostream& operator << (std::ostream& io, Test::SimpleVector const& data);
    extern std::ostream& operator << (std::ostream& io, Test::ComplexVector const& data);
    extern std::ostream& operator << (std::ostream& io, Test::ComplexArray const& data);
}

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
bool generic_corba_test(T const& testValue)
{
    boost::intrusive_ptr< ConstantDataSource<T> > source(new ConstantDataSource<T>(testValue));
    source->ref();

    cerr << "- testing CORBA marshalling/unmarshalling ..." << endl;
    { CORBA::Any* result = reinterpret_cast<CORBA::Any*>(source->createBlob(ORO_CORBA_PROTOCOL_ID));
        ValueDataSource<T>* reader = new ValueDataSource<T>();
        reader->ref();
        reader->updateBlob(ORO_CORBA_PROTOCOL_ID, result);
        T value = reader->get();
        if (!(value == testValue))
        {
            cerr << "error. Expected\n\n" << testValue <<
                "\n\nand got\n\n" << value << endl;
            return false;
        }
    }
    cerr << "  done" << endl;
    return true;
}
#endif

template<typename T>
bool generic_type_handling_test(std::string const& name, T const& testValue, TypeInfo const& ti)
{
    cerr << "- testing decomposition to XML formats" << endl;
    ConstantDataSource<T>* source
        = new ConstantDataSource<T>(testValue);
    source->ref();

    PropertyBag bag;
    if (!ti.decomposeType(source, bag))
    {
        cerr << "  decomposition failed" << endl;
        return false;
    }
    cerr << "  done decomposition" << endl;

    // First, save it into XML. The Ruby test case will compare that to an
    // expected XML document
    std::ofstream xml_file((name + ".xml").c_str());
    XMLMarshaller<std::ostream> xml_output(xml_file);
    xml_output.serialize(bag);
    cerr << "  done XML serialization" << endl;

    // Now, marshal it to the standard Orocos format, reload it and compare
    // the result
    PropertyMarshaller cpf_output(name + ".cpf");
    cpf_output.serialize(bag);
    cpf_output.flush();
    cerr << "  done CPF serialization" << endl;

    cerr << "- testing unmarshalling from XML" << endl;
    PropertyBag input_bag;
    PropertyDemarshaller cpf_input(name + ".cpf");
    cpf_input.deserialize(input_bag);

    { ValueDataSource<T>* reader = new ValueDataSource<T>();
        reader->ref();
        Property<PropertyBag> bag("", "", input_bag);
        if (!ti.composeType(bag.getDataSource(), reader))
        {
            cerr << "cannot recompose type" << endl;
            return false;
        }

        T value = reader->get();
        if (!(value == testValue))
        {
            cerr << "error. Expected\n\n" << testValue <<
                "\n\nand got\n\n" << value << endl;
            return false;
        }
    }
    cerr << "  done" << endl;

#ifdef WITH_CORBA
    return generic_corba_test(testValue);
#else
    return true;
#endif
}

bool test_base_types()
{
    cerr << "Test handling of BaseTypes" << endl;
    TypeInfo* type = assert_typeinfo_is_available("/Test/BaseTypes");

    // Create a structure of type Test::BaseTypes, inject it into Orocos and check
    // that it is able to generate a XML representation of it
    Test::BaseTypes testValue;
    testValue.v0 = true;
    testValue.v1 = -100;
    testValue.v2 = 200;
    testValue.v3 = -1024;
    testValue.v4 = 40000;
    testValue.v5 = -100000;
    testValue.v6 = 3000000000UL;
    testValue.e   = Test::VALUE_20;
    for (int i = 0; i < 20; ++i)
	testValue.a[i] = 'a' + i;

    return generic_type_handling_test("basic", testValue, *type);
}

bool test_handling_of_invalid_enum_values()
{
    cerr << "test_handling_of_invalid_enum_values" << endl;
    TypeInfo* type = assert_typeinfo_is_available("/Test/BaseTypes");
    Test::BaseTypes testValue;
    memset(&testValue, 0, sizeof(testValue));
    testValue.e   = (Test::BASIC_ENUM)999;

    boost::intrusive_ptr< ConstantDataSource<Test::BaseTypes> > source(new ConstantDataSource<Test::BaseTypes>(testValue));
    source->ref();

    // Decomposition should fail
    cerr << "- testing that decomposition is forbidden" << endl;
    PropertyBag bag;
    if (type->decomposeType(source, bag))
    {
        cerr << "  FAILED" << endl;
        return false;
    }

    // TODO: test that CORBA fails as well
    return true;
}

bool test_64bit_handling()
{
    cerr << "Testing 64bit handling" << endl;
    TypeInfo* type = assert_typeinfo_is_available("/Test/Test64BitHandling");

    Test::Test64BitHandling testValue;
    memset(&testValue, 0, sizeof(testValue));
    testValue.base.v0 = true;
    testValue.base.v1 = -100;
    testValue.base.v2 = 200;
    testValue.base.v3 = -1024;
    testValue.base.v4 = 40000;
    testValue.base.v5 = -100000;
    testValue.base.v6 = 3000000000UL;
    testValue.base.e   = Test::VALUE_20;
    for (int i = 0; i < 20; ++i)
	testValue.base.a[i] = 'a' + i;
    testValue.ll  = -(1LL << 40);
    testValue.ull = -(1LL << 40);

    boost::intrusive_ptr< ConstantDataSource<Test::Test64BitHandling> > source(new ConstantDataSource<Test::Test64BitHandling>(testValue));
    source->ref();

    // Decomposition should fail
    cerr << "- testing that decomposition is forbidden" << endl;
    PropertyBag bag;
    if (type->decomposeType(source, bag))
    {
        cerr << "  FAILED" << endl;
        return false;
    }

    // But CORBA marshalling should work just fine
#ifdef WITH_CORBA
    return generic_corba_test(testValue);
#else
    return true;
#endif
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
    RTT::Typekit::Import( orogen_typekits::simpleTypekit );
#ifdef WITH_CORBA
    RTT::Typekit::Import( orogen_typekits::simpleCorbaTransport );
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

