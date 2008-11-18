#include "TestToolkitTypes.hpp"
#include "TestToolkit.hpp"

#ifdef WITH_CORBA
#include <omniORB4/CORBA.h>
#include <rtt/corba/CorbaLib.hpp>
#include "build/.orogen/toolkit/TestToolkitC.h"
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

int ORO_main(int argc, char** argv)
{
    log().setLogLevel( Logger::Debug );
    RTT::Toolkit::Import( Test::Toolkit );

    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();

    TypeInfo* type = ti->type("/Test/Simple");
    if (! type)
    {
	cerr << "cannot find /Test/Simple in the type info repository" << endl;
	return 1;
    }

    // Create a structure of type Test::Simple, inject it into Orocos and check
    // that it is able to generate a XML representation of it
    Test::Simple value = { 10, "" };
    for (int i = 0; i < 20; ++i)
	value.b[i] = 'a' + i;

    ConstantDataSource<Test::Simple>* source
        = new ConstantDataSource<Test::Simple>(value);
    source->ref();

    PropertyBag bag;
    type->decomposeType(source, bag);

    // First, save it into XML. The Ruby test case will compare that to an
    // expected XML document
    std::ofstream xml_file("test_toolkit.xml");
    XMLMarshaller<std::ostream> xml_output(xml_file);
    xml_output.serialize(bag);

    // Now, marshal it to the standard Orocos format, reload it and compare
    // the result
    PropertyMarshaller cpf_output("test_toolkit.cpf");
    cpf_output.serialize(bag);
    cpf_output.flush();

    PropertyBag input_bag;
    PropertyDemarshaller cpf_input("test_toolkit.cpf");
    cpf_input.deserialize(input_bag);

    int input_a = Property<int>(input_bag.getItem(0)).value();
    if (input_a != 10)
    {
	cerr << "input.a == " << input_a << ", 10 expected" << endl;
	return 1;
    }

#ifdef WITH_CORBA
    std::cerr << "Testing the CORBA part ..." << std::endl;
    // And now the CORBA part. First marshalling ...
    { CORBA::Any* result
        = reinterpret_cast<CORBA::Any*>(source->createBlob(ORO_CORBA_PROTOCOL_ID));

        Test::Corba::Simple *input;
        (*result) >>= input;
        if (input->a != 10)
        {
            cerr << "error in CORBA marshalling" << endl;
            cerr << "input.a == " << input->a << ", 10 expected" << endl;
            return 1;
        }

        for (int i = 0; i < 20; ++i)
        {
            if (input->b[i] != 'a' + i)
            {
                cerr << "error in CORBA marshalling" << endl;
                cerr << "array[" << i << "] == " << input->b[i] << ", " << 'a' + i << " expected" << endl;
                return 1;
            }
        }
    }

    // And then unmarshalling
    { CORBA::Any* result = reinterpret_cast<CORBA::Any*>(source->createBlob(ORO_CORBA_PROTOCOL_ID));
        ValueDataSource<Test::Simple>* reader =
            new ValueDataSource<Test::Simple>();
        reader->ref();
        reader->updateBlob(ORO_CORBA_PROTOCOL_ID, result);
        Test::Simple value = reader->get();

        if (value.a != 10)
        {
            cerr << "error in CORBA marshalling" << endl;
            cerr << "input.a == " << input_a << ", 10 expected" << endl;
            return 1;
        }

        for (int i = 0; i < 20; ++i)
        {
            if (value.b[i] != 'a' + i)
            {
                cerr << "error in CORBA marshalling" << endl;
                cerr << "array[" << i << "] == " << value.b[i] << ", " << 'a' + i << " expected" << endl;
                return 1;
            }
        }
    }

#endif

    return 0;
}

