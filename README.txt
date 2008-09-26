= Orogen: easy component building for Orocos/RTT

The Orocos/Real Time Toolkit is a C++ library allowing to build component-based
systems (http://www.orocos.org/rtt). The orogen tool allows to create new
components easily. This readme presents the tools

== Orogen specification
An orogen specification is actually Ruby code, but you don't really need to
know Ruby to write one. This section presents the different parts that the
specification can contain, and also provides the basic options that you can use.

The four parts an Orogen specification are made of are:
* component-level attributes, like its name, version and dependencies. More on
  that later
* a _toolkit_ specification. In Orocos/RTT, toolkits are libraries which declares
  new data types into the Orocos type system, allowing for instance to use them
  in the multi-process layers (i.e. CORBA). With orogen, the only thing you need to
  do is write a C-like code defining the structure, orogen does the rest.
* various <i>task contexts</i> specifications. The TaskContext subclasses are
  in Orocos the actual classes which contain the "functionality" of the
  component. They manage the different state machines and the different methods
  which will *act* during the component's execution. The set of task contexts
  form a <i>task library</i> that can be reused in other orogen components.
* a _deployment_ specification. This allows to build an executable in which
  some of the TaskContext are instanciated, linked to an Activity and
  (optionally) started.

=== Component-level attributes
First, you <b>have to</b> declare the component name and version. This is mandatory.

  name "my_component"
  version "0.1"

You can also declare that your component depends on other libraries, task
libraries and toolkits (task libraries and toolkits have to be
Orogen-generated). This is done through the use of the pkg-config tool, so you
need your external dependencies to provide the necessary files (and those files
to be in a directory listed in the +PKG_CONFIG_PATH+ environment variable).
Orogen generates and installs those files for task libraries and toolkits.

  using_library "libname"
  using_task_library "component_name"
  using_toolkit "component_name"

More generally, the statements that appear at the top level of the orogen file
are actually methods call on an instance of Orocos::Generation::Component. See
the documentation of that class for more details.

=== Toolkit definition

The presence of a toolkit in the component is declared with
  toolkit do
    <i>toolkit definition</i>
  end

Where the <i>toolkit definition</i> is a set of statements, each statement
being an instance method of Orocos::Generation::Toolkit.

The most useful statement is
  load "my_header_file.h"

Two less-often used statements are:
  preload "system_file.h"
  internal_dependency "pkg-name"

See the documentation of Orocos::Generation::Toolkit for more details.

=== Task context definition

The component can define a <em>task library</em>, i.e. a set of TaskContext
classes. Each of those are declared through the
Orocos::Generation::Component#task_context call:

  task_context "ClassName" do
    ... task context specification ...
  end

Example of the definition of various objects:

  task_context "ExampleTask" do
    # Create a Orocos::Generation::Property object to describe an orocos property
    property('device_name', 'std::string', '/dev/ttyS1').
      doc 'the device name to connect to (a string property with default value)'

    # The generated class starts in PreOperational state, configureHook()
    # checks that 'device_name' is valid
    needs_configuration

    # Create a Orocos::Generation::Method object to describe an orocos command
    method('reset').
      returns('int').
      doc 'resets the device. Returns true on success'

    # Create a Orocos::Generation::Command object to describe an orocos command
    command('mean_value').
      argument('duration', 'double', 'the duration of the mean in seconds')

    # Create Orocos::Generation::DataPort object to describe orocos data ports
    data_port('dout', 'double', 'w').
      doc 'an output non-buffered port of type double'
    data_port('din', '/std/string', 'r').
      doc 'an input non-buffered port of type std::string'
    # Create Orocos::Generation::BufferPort object to describe orocos data ports
    buffer_port('bout', 'double', 'w', 10).
      doc 'an output buffered port of type double'
    buffer_port('bin', 'double', 'r').
      doc 'an input buffered port of type double'
  end

The task context specification is a set of method calls on the
Orocos::Generation::TaskContext instance representing the new task context. See
the documentation of that class (and of the various classes mentioned in the above
example) for more details.

=== Static deployment

Orogen has the ability to build a binary in which some tasks are deployed (i.e.
associated with Activities). Such a deployment is declared using
  static_deployment do
    ... deployment specification ...
  end

The deployment specification is represented by a
Orocos::Generation::StaticDeployment instance. See the documentation of that
class for more details.

=== CORBA support

Orogen has support for the CORBA transport in the following areas:
* it is able to create all the necessary definitions so that the generated
  toolkits allow passing types through CORBA
* it is able to generate the code necessary for a static deployment to be
  accessible through CORBA.

= Workflow

1. Initial generation
   Orogen is basically a code generator. You write a
   <tt>my_component.orogen</tt> file, which is actually Ruby code, and orogen generates
   two set of things:
   * purely generated code, that should not be modified by you. This code is saved in the
     <tt>.orogen</tt> subdirectory.
   * user-modifiable code, where you put your own part of the implementation.
     Template for this code is generated once.

2. Modifications
   The separation between user-modifiable and autogenerated code allows you to
   modify the component specification (the +.orogen+ file) without having the tool
   destroy your code. It is therefore possible to change the +.orogen+ file, but
   you will sometime have to adapt the user-visible part of the code to reflect 
   those changes -- or the component will not compile anymore.

   After each generation, updated templates based on the new specification are
   saved in the templates/ directory of the user-visible part.

== Running the orogen tool

The general signature (and most of the time the only thing you need to do) is
  orogen specification.orogen

and you are done !. The only often-used option to orogen is the
<tt>--corba</tt> option, which enables corba support in the generated code
(i.e. in toolkits and static deployments). Corba can be enabled/disabled
directly in the specification by using the various #enable_corba and
#disable_corba methods, but it is discouraged as Orocos allows to use
transparently the same task contexts in both CORBA-enabled and CORBA-disabled
environments.

Two other options can also generally be useful: <tt>--clean</tt> and
<tt>--really-clean</tt>. The first one simply removes the autogenerated part of
the component, namely the <tt>.orogen</tt> and <tt>templates</tt> directories.
The second one also removes the files in the user part that are identical to
the template (i.e. have not been changed by the user).

== Building Orocos component
Once you have specified your component, generated it, it needs to be compiled.
Orogen generates a set of CMake (http://www.cmake.org) files, necessary to build
and install the component
