#include <rtt/Toolkit.hpp>
#include "opaqueToolkitTypes.hpp"
#include "opaqueToolkit.hpp"
#include "opaque.h"

#ifdef WITH_CORBA
#include <omniORB4/CORBA.h>
#include <rtt/corba/CorbaLib.hpp>
#include "build/.orogen/toolkit/opaqueToolkitC.h"
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

using namespace RTT;
using std::cerr;
using std::endl;

static const int TYPELIB_MARSHALLER_ID = 42;
namespace NotOrogenCompatible {
    extern std::ostream& operator << (std::ostream& io, Point2D const& data);
}
namespace TestOpaque {
    extern std::ostream& operator << (std::ostream& io, Point2D const& data);
    extern std::ostream& operator << (std::ostream& io, Position const& data);
    extern std::ostream& operator << (std::ostream& io, Position_m const& data);
}
namespace std {
    extern std::ostream& operator << (std::ostream& io, std::vector<float> const& data);
    extern std::ostream& operator << (std::ostream& io, std::vector<int> const& data);
}
template<typename T>
T identity(T const& value) { return value; }

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

    PropertyBag bag;
    ti.decomposeType(source, bag);

    // First, save it into XML. The Ruby test case will compare that to an
    // expected XML document
    std::ofstream xml_file((name + ".xml").c_str());
    XMLMarshaller<std::ostream> xml_output(xml_file);
    xml_output.serialize(bag);

    // Now, marshal it to the standard Orocos format, reload it and compare
    // the result
    PropertyMarshaller cpf_output(name + ".cpf");
    cpf_output.serialize(bag);
    cpf_output.flush();

    PropertyBag input_bag;
    PropertyDemarshaller cpf_input(name + ".cpf");
    cpf_input.deserialize(input_bag);

    cerr << "Testing PropertyBag composition" << endl;
    { ValueDataSource<T>* reader = new ValueDataSource<T>();
        reader->ref();
        Property<PropertyBag> bag("", "", input_bag);
        if (!ti.composeType(bag.getDataSource(), reader))
        {
            cerr << "cannot recompose type" << endl;
            return false;
        }

        T value = reader->get();
        if (!(get_test_value(value) == get_test_value(testValue)))
        {
            cerr << "error. Expected\n\n" << get_test_value(testValue) <<
                "\n\nand got\n\n" << get_test_value(value) << endl;
        }
    }

#ifdef WITH_CORBA
    std::cerr << "Testing CORBA marshalling/unmarshalling ..." << std::endl;
    { CORBA::Any* result = reinterpret_cast<CORBA::Any*>(source->createBlob(ORO_CORBA_PROTOCOL_ID));
        ValueDataSource<T>* reader = new ValueDataSource<T>();
        reader->ref();
        reader->updateBlob(ORO_CORBA_PROTOCOL_ID, result);
        T value = reader->get();
        if (!(get_test_value(value) == get_test_value(testValue)))
        {
            cerr << "error. Expected\n\n" << get_test_value(testValue) <<
                "\n\nand got\n\n" << get_test_value(value) << endl;
        }
    }
#endif
    return true;
}

bool test_plain_opaque()
{
    cerr << "======== Testing plain opaque handling ========" << endl;

    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    TypeInfo* type = ti->type("/NotOrogenCompatible/Point2D");
    if (! type)
    {
	cerr << "cannot find /NotOrogenCompatible/Point2D in the type info repository" << endl;
	return 1;
    }

    NotOrogenCompatible::Point2D testValue(10, 20);
    generic_type_handling_test("opaque", testValue, *type, &identity<NotOrogenCompatible::Point2D> );

    // Now, try the marshalling thing
    std::cerr << "Testing the Pocosim marshalling ..." << std::endl;
    {
        ConstantDataSource<NotOrogenCompatible::Point2D>* source =
            new ConstantDataSource<NotOrogenCompatible::Point2D>(testValue);
        source->ref();
        std::vector<uint8_t>* result =
            reinterpret_cast< std::vector<uint8_t>* >(source->createBlob(TYPELIB_MARSHALLER_ID));

        if (sizeof(TestOpaque::Point2D) != result->size())
        {
            cerr << "error in Pocosim marshalling" << endl;
            cerr << "resulting buffer is of size " << result->size() << ", " << sizeof(TestOpaque::Point2D) << " expected" << endl;
            return false;
        }

        TestOpaque::Point2D p = *reinterpret_cast<TestOpaque::Point2D*>(&(*result)[0]);
        if (p.padding != 100 || p.x != testValue.x() || p.y != testValue.y())
        {
            cerr << "error in Pocosim marshalling" << endl;
            cerr << "expected\n\n" << testValue <<
                "\n\ngot\n\n" << p << endl;
            return false;
        }
    }

    return true;
}

bool test_composed_opaque()
{
    cerr << "======== Testing opaque field in struct ========" << endl;

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

    generic_type_handling_test("composed_opaque", testValue, *type, &identity<TestOpaque::Position> );

    std::cerr << "Testing the Pocosim marshalling ..." << std::endl;
    {
        ConstantDataSource<TestOpaque::Position>* source
            = new ConstantDataSource<TestOpaque::Position>(testValue);
        source->ref();
        std::vector<uint8_t>* result =
            reinterpret_cast< std::vector<uint8_t>* >(source->createBlob(TYPELIB_MARSHALLER_ID));

        if (sizeof(TestOpaque::Position_m) != result->size())
        {
            cerr << "error in Pocosim marshalling" << endl;
            cerr << "resulting buffer is of size " << result->size() << ", " << sizeof(TestOpaque::Point2D) << " expected" << endl;
            return false;
        }

        TestOpaque::Position_m p = *reinterpret_cast<TestOpaque::Position_m*>(&(*result)[0]);
        if (p.timestamp != testValue.timestamp ||
                p.p.x != testValue.p.x() ||
                p.p.y != testValue.p.y())
        {
            cerr << "error in pocosim marshalling for TestOpaque::Position_m" << endl;
            return false;
        }
    }

    return true;
}

template<typename T>
T get_ptr_content(boost::shared_ptr<T> const& left)
{ return *left; }

template<typename T>
T get_ro_ptr_content(RTT::ReadOnlyPointer<T> const& left)
{ return *left; }

bool test_shared_pointer()
{
    cerr << "======== Testing shared pointer ========" << endl;

    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    TypeInfo* type = ti->type("/boost/shared_ptr</std/vector</float>>");
    if (! type)
    {
	cerr << "cannot find /boost/shared_ptr</std/vector</float>> in the type info repository" << endl;
	return 1;
    }

    boost::shared_ptr< std::vector<float> > testValue( new std::vector<float> );
    std::vector<float>& data = *testValue;

    for (int i = 0; i < 10; ++i)
        data.push_back(i);


    generic_type_handling_test("shared_ptr__opaque_type", testValue, *type, &get_ptr_content< std::vector<float> >);

    std::cerr << "Testing the Pocosim marshalling ..." << std::endl;
    {
        ConstantDataSource<boost::shared_ptr< std::vector<float> > >* source
            = new ConstantDataSource<boost::shared_ptr< std::vector<float> > >(testValue);
        source->ref();
        std::vector<uint8_t>* result =
            reinterpret_cast< std::vector<uint8_t>* >(source->createBlob(TYPELIB_MARSHALLER_ID));

        uint64_t size = *reinterpret_cast<uint64_t*>(&(*result)[0]);
        if (size != (*testValue).size())
        {
            cerr << "error in pocosim marshalling for boost::shared_ptr< std::vector<float> >" << endl;
            return false;
        }

        float* data = reinterpret_cast<float*>(&(*result)[sizeof(size)]);
        for (int i = 0; i < size; ++i)
            if (data[i] != (*testValue)[i])
            {
            cerr << "error in pocosim marshalling for boost::shared_ptr< std::vector<float> >" << endl;
            return false;
            }
    }

    return true;
}

bool test_shared_ptr_shortcut()
{
    cerr << "======== Testing shared pointer (from #shared_ptr) ========" << endl;

    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    TypeInfo* type = ti->type("/boost/shared_ptr</std/vector</int>>");
    if (! type)
    {
	cerr << "cannot find /boost/shared_ptr</std/vector</int>> in the type info repository" << endl;
	return 1;
    }

    boost::shared_ptr< std::vector<int> > testValue( new std::vector<int> );
    std::vector<int>& data = *testValue;

    for (int i = 0; i < 10; ++i)
        data.push_back(i);


    generic_type_handling_test("shared_ptr__shared_ptr", testValue, *type, &get_ptr_content< std::vector<int> >);

    std::cerr << "Testing the Pocosim marshalling ..." << std::endl;
    {
        ConstantDataSource<boost::shared_ptr< std::vector<int> > >* source
            = new ConstantDataSource<boost::shared_ptr< std::vector<int> > >(testValue);
        source->ref();
        std::vector<uint8_t>* result =
            reinterpret_cast< std::vector<uint8_t>* >(source->createBlob(TYPELIB_MARSHALLER_ID));

        uint64_t size = *reinterpret_cast<uint64_t*>(&(*result)[0]);
        if (size != (*testValue).size())
        {
            cerr << "error in pocosim marshalling for boost::shared_ptr< std::vector<int> >" << endl;
            return false;
        }

        int* data = reinterpret_cast<int*>(&(*result)[sizeof(size)]);
        for (int i = 0; i < size; ++i)
            if (data[i] != (*testValue)[i])
            {
            cerr << "error in pocosim marshalling for boost::shared_ptr< std::vector<int> >" << endl;
            return false;
            }
    }

    return true;
}

bool test_ro_ptr()
{
    cerr << "======== Testing ReadOnlyPointer (from #ro_ptr) ========" << endl;

    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    TypeInfo* type = ti->type("/RTT/ReadOnlyPointer</std/vector</int>>");
    if (! type)
    {
	cerr << "cannot find /RTT/ReadOnlyPointer</std/vector</int>> in the type info repository" << endl;
	return 1;
    }

    std::vector<int>* data = new std::vector<int>();
    for (int i = 0; i < 10; ++i)
        data->push_back(i);

    RTT::ReadOnlyPointer< std::vector<int> > testValue(data);


    generic_type_handling_test("readonlypointer", testValue, *type, &get_ro_ptr_content< std::vector<int> >);

    std::cerr << "Testing the Pocosim marshalling ..." << std::endl;
    {
        ConstantDataSource<RTT::ReadOnlyPointer< std::vector<int> > >* source
            = new ConstantDataSource<RTT::ReadOnlyPointer< std::vector<int> > >(testValue);
        source->ref();
        std::vector<uint8_t>* result =
            reinterpret_cast< std::vector<uint8_t>* >(source->createBlob(TYPELIB_MARSHALLER_ID));

        uint64_t size = *reinterpret_cast<uint64_t*>(&(*result)[0]);
        if (size != (*testValue).size())
        {
            cerr << "error in pocosim marshalling for RTT::ReadOnlyPointer< std::vector<int> >" << endl;
            return false;
        }

        int* data = reinterpret_cast<int*>(&(*result)[sizeof(size)]);
        for (int i = 0; i < size; ++i)
            if (data[i] != (*testValue)[i])
            {
            cerr << "error in pocosim marshalling for RTT::ReadOnlyPointer< std::vector<int> >" << endl;
            return false;
            }
    }

    return true;
}


int ORO_main(int argc, char** argv)
{
    log().setLogLevel( Logger::Debug );
    RTT::Toolkit::Import( orogen_toolkits::opaqueToolkit );

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

