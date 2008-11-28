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

namespace NotOrogenCompatible {
    extern std::ostream& operator << (std::ostream& io, Point2D const& data);
}
namespace TestOpaque {
    extern std::ostream& operator << (std::ostream& io, Point2D const& data);
    extern std::ostream& operator << (std::ostream& io, Position const& data);
    extern std::ostream& operator << (std::ostream& io, Position_m const& data);
}

template<typename T>
bool generic_type_handling_test(std::string const& name, T const& testValue, TypeInfo const& ti)
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
        if (!(value == testValue))
        {
            cerr << "error. Expected\n\n" << testValue <<
                "\n\nand got\n\n" << value << endl;
        }
    }

#ifdef WITH_CORBA
    std::cerr << "Testing CORBA marshalling/unmarshalling ..." << std::endl;
    { CORBA::Any* result = reinterpret_cast<CORBA::Any*>(source->createBlob(ORO_CORBA_PROTOCOL_ID));
        ValueDataSource<T>* reader = new ValueDataSource<T>();
        reader->ref();
        reader->updateBlob(ORO_CORBA_PROTOCOL_ID, result);
        T value = reader->get();
        if (!(value == testValue))
        {
            cerr << "error. Expected\n\n" << testValue <<
                "\n\nand got\n\n" << value << endl;
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
    generic_type_handling_test("opaque", testValue, *type);

    // Now, try the marshalling thing
    std::cerr << "Testing the Pocosim marshalling ..." << std::endl;
    {
        ConstantDataSource<NotOrogenCompatible::Point2D>* source =
            new ConstantDataSource<NotOrogenCompatible::Point2D>(testValue);
        source->ref();
        std::vector<uint8_t>* result =
            reinterpret_cast< std::vector<uint8_t>* >(source->createBlob(opaque::ORO_UNTYPED_PROTOCOL_ID));

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

    generic_type_handling_test("composed_opaque", testValue, *type);

    std::cerr << "Testing the Pocosim marshalling ..." << std::endl;
    {
        ConstantDataSource<TestOpaque::Position>* source
            = new ConstantDataSource<TestOpaque::Position>(testValue);
        source->ref();
        std::vector<uint8_t>* result =
            reinterpret_cast< std::vector<uint8_t>* >(source->createBlob(opaque::ORO_UNTYPED_PROTOCOL_ID));

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
            cerr << "error in pocosim marshalling" << endl;
            cerr << "expected\n\n" << testValue << "\n\ngot\n\n" << p << endl;
            return false;
        }
    }

    return true;
}

int ORO_main(int argc, char** argv)
{
    log().setLogLevel( Logger::Debug );
    RTT::Toolkit::Import( opaque::Toolkit );

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

    return 0;
}

