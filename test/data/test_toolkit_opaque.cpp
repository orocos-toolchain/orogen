#include "TestOpaqueToolkitTypes.hpp"
#include "TestOpaqueToolkit.hpp"
#include "opaque.h"

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

int ORO_main(int argc, char** argv)
{
    log().setLogLevel( Logger::Debug );
    RTT::Toolkit::Import( TestOpaque::Toolkit );

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
    }

    double input_a = Property<double>(input_bag.getItem(1)).value();
    double input_b = Property<double>(input_bag.getItem(2)).value();
    if (input_a != 10 || input_b != 20)
    {
        cerr << "error in property marshalling/demarshalling" << endl;
	cerr << "input.a == " << input_a << ", 10 expected" << endl;
	cerr << "input.b == " << input_b << ", 20 expected" << endl;
	return 1;
    }

    return 0;
}

