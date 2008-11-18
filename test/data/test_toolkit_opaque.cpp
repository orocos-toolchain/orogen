#include "TestOpaqueToolkitTypes.hpp"
#include "TestOpaqueToolkit.hpp"
#include "opaque.h"

#ifdef WITH_CORBA
#include <omniORB4/CORBA.h>
#include <rtt/corba/CorbaLib.hpp>
#include "build/.orogen/toolkit/TestOpaqueToolkitC.h"
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

bool test_plain_opaque()
{
    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    TypeInfo* type = ti->type("/NotOrogenCompatible/Point2D");
    if (! type)
    {
	cerr << "cannot find /NotOrogenCompatible/Point2D in the type info repository" << endl;
	return 1;
    }

    // Create a structure of type Test::Simple, inject it into Orocos and check
    // that it is able to generate a XML representation of it
    NotOrogenCompatible::Point2D value(10, 20);
    ConstantDataSource<NotOrogenCompatible::Point2D>* source =
        new ConstantDataSource<NotOrogenCompatible::Point2D>(value);
    source->ref();

    PropertyBag bag;
    type->decomposeType(source, bag);

    // First, save it into XML. The Ruby test case will compare that to an
    // expected XML document
    std::ofstream xml_file("test_toolkit_opaque.xml");
    XMLMarshaller<std::ostream> xml_output(xml_file);
    xml_output.serialize(bag);

    // Now, marshal it to the standard Orocos format, reload it and compare
    // the result
    PropertyMarshaller cpf_output("test_toolkit_opaque.cpf");
    cpf_output.serialize(bag);
    cpf_output.flush();

    PropertyBag input_bag;
    PropertyDemarshaller cpf_input("test_toolkit_opaque.cpf");
    cpf_input.deserialize(input_bag);

    if (input_bag.size() != 3)
    {
        cerr << "error in property marshalling/demarshalling" << endl;
        cerr << "expected 3 elements, got " << input_bag.size() << endl;
        return false;
    }

    double input_a = Property<double>(input_bag.getItem(1)).value();
    double input_b = Property<double>(input_bag.getItem(2)).value();
    if (input_a != 10 || input_b != 20)
    {
        cerr << "error in property marshalling/demarshalling" << endl;
	cerr << "input.a == " << input_a << ", 10 expected" << endl;
	cerr << "input.b == " << input_b << ", 20 expected" << endl;
	return false;
    }

#ifdef WITH_CORBA
    std::cerr << "Testing the CORBA part ..." << std::endl;
    // And now the CORBA part. First marshalling ...
    { CORBA::Any_var result = reinterpret_cast<CORBA::Any*>(source->createBlob(ORO_CORBA_PROTOCOL_ID));

        result >>= input_a;
        result >>= input_b;
        if (input_a != 10 || input_b != 20)
        {
            cerr << "error in CORBA marshalling" << endl;
            cerr << "input.a == " << input_a << ", 10 expected" << endl;
            cerr << "input.b == " << input_b << ", 20 expected" << endl;
            return false;
        }
    }

    // And then unmarshalling
    { CORBA::Any* result = reinterpret_cast<CORBA::Any*>(source->createBlob(ORO_CORBA_PROTOCOL_ID));
        ValueDataSource<NotOrogenCompatible::Point2D>* reader =
            new ValueDataSource<NotOrogenCompatible::Point2D>();
        reader->ref();
        reader->updateBlob(ORO_CORBA_PROTOCOL_ID, result);
        NotOrogenCompatible::Point2D value = reader->get();

        if (value.x() != 10 || value.y() != 20)
        {
            cerr << "error in CORBA unmarshalling" << endl;
            cerr << "value.x() == " << value.x() << ", 10 expected" << endl;
            cerr << "value.y() == " << value.y() << ", 20 expected" << endl;
            return false;
        }
    }
#endif
    return true;
}

bool check_position_value(std::string const& place, double timestamp, double x, double y)
{
    if (timestamp != 10 || x != 20 || y != 30)
    {
        cerr << "error in " << place << endl;
        cerr << "timestamp == " << timestamp << ", 10 expected" << endl;
        cerr << "p.x == " << x << ", 20 expected" << endl;
        cerr << "p.y == " << y << ", 30 expected" << endl;
        return false;
    }
    return true;
}

bool test_composed_opaque()
{
    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    TypeInfo* type = ti->type("/TestOpaque/Position");
    if (! type)
    {
	cerr << "cannot find /TestOpaque/Position in the type info repository" << endl;
	return 1;
    }

    // Create a structure of type Test::Simple, inject it into Orocos and check
    // that it is able to generate a XML representation of it
    TestOpaque::Position test_value;
    test_value.timestamp = 10;
    test_value.p.x() = 20;
    test_value.p.y() = 30;

    ConstantDataSource<TestOpaque::Position>* source =
        new ConstantDataSource<TestOpaque::Position>(test_value);
    source->ref();

    PropertyBag bag;
    type->decomposeType(source, bag);

    // First, save it into XML. The Ruby test case will compare that to an
    // expected XML document
    std::ofstream xml_file("test_toolkit_composed_opaque.xml");
    XMLMarshaller<std::ostream> xml_output(xml_file);
    xml_output.serialize(bag);

    // Now, marshal it to the standard Orocos format, reload it and compare
    // the result
    PropertyMarshaller cpf_output("test_toolkit_composed_opaque.cpf");
    cpf_output.serialize(bag);
    cpf_output.flush();

    PropertyBag input_bag;
    PropertyDemarshaller cpf_input("test_toolkit_composed_opaque.cpf");
    cpf_input.deserialize(input_bag);

    if (input_bag.size() != 4)
    {
        cerr << "error in property marshalling/demarshalling" << endl;
        cerr << "expected 4 elements, got " << input_bag.size() << endl;
    }

    double timestamp = Property<double>(input_bag.getItem(0)).value();
    double x = Property<double>(input_bag.getItem(2)).value();
    double y = Property<double>(input_bag.getItem(3)).value();
    if (!check_position_value("property demarshalling", timestamp, x, y))
        return false;

#ifdef WITH_CORBA
    std::cerr << "Testing the CORBA part ..." << std::endl;
    // And now the CORBA part. First marshalling ...
    { CORBA::Any_var result = reinterpret_cast<CORBA::Any*>(source->createBlob(ORO_CORBA_PROTOCOL_ID));
        TestOpaque::Corba::Position* value;
        TestOpaque::Corba::Point2D*  value_p;
        result  >>= value;
        value->p >>= value_p;

        if (!check_position_value("CORBA marshalling", value->timestamp, value_p->x, value_p->y))
            return false;
    }

    // And then unmarshalling
    { CORBA::Any* result = reinterpret_cast<CORBA::Any*>(source->createBlob(ORO_CORBA_PROTOCOL_ID));
        ValueDataSource<TestOpaque::Position>* reader =
            new ValueDataSource<TestOpaque::Position>();
        reader->ref();
        reader->updateBlob(ORO_CORBA_PROTOCOL_ID, result);
        TestOpaque::Position value = reader->get();

        if (!check_position_value("CORBA demarshalling", value.timestamp, value.p.x(), value.p.y()))
            return false;
    }
#endif
    return true;
}

int ORO_main(int argc, char** argv)
{
    log().setLogLevel( Logger::Debug );
    RTT::Toolkit::Import( TestOpaque::Toolkit );

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

