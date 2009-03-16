require 'orogen/test'

class TC_GenerationTasks < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'generation', 'data' )

    def test_generation_validation
	component = Component.new

        # Should raise because there is no name
        assert_raises(ArgumentError) { component.generate }

        component.name "test"
        # Should raise because there is no orogen file
        assert_raises(ArgumentError) { component.generate }

        component.instance_variable_set(:@deffile, "bla.orogen")

        create_wc("tasks/generation_validation")
        in_wc do
            component.generate
        end
    end

    def test_task_context
	component = Component.new
	component.name 'test'

	name = 'task_name'
	doc  = 'task doc'

	task = component.task_context(name)
	assert_raises(ArgumentError) { component.task_context(name) }

	assert_kind_of(Generation::TaskContext, task)
	assert_equal("test::#{name}", task.name)

	# Check name validation
	assert_raises(ArgumentError) { component.task_context("bla bla") }
	assert_raises(ArgumentError) { component.task_context("bla(bla") }
	assert_raises(ArgumentError) { component.task_context("bla!bla") }
	assert_raises(ArgumentError) { component.task_context("bla/bla") }

        create_wc("tasks/task_context")
        compile_wc(component)
    end

    def test_property
	component = Component.new
	component.name 'test'

	name = 'property_name'
	doc  = 'property doc'
	type = '/std/vector</double>'

	task = component.task_context(name)

	property = task.property(name, type).
	    doc(doc)
	assert_raises(ArgumentError) { task.property(name) }
	assert_raises(ArgumentError) { task.property("bla bla") }

	assert_kind_of(Generation::Property, property)
	assert_equal(name, property.name)
	assert_equal(type, property.type.full_name)
	assert_equal(doc,  property.doc)

        create_wc("tasks/property")
	compile_wc(component)
    end

    def test_task_method
	component = Component.new
	component.name 'test'

	task = component.task_context("Task")

	meth = task.method("MethodName").
	    doc("the method to test")
	assert_equal([meth], task.self_methods)

	assert_kind_of(Generation::Method, meth)
	assert_equal("MethodName", meth.name)
	assert_equal("methodName", meth.method_name)
	assert_equal(nil,  meth.return_type)
	assert_equal("the method to test",  meth.doc)
	assert(meth.arguments.empty?)
	assert_equal("void()", meth.signature)

	meth.method_name "another_method_name"
	assert_equal("MethodName", meth.name)
	assert_equal("another_method_name", meth.method_name)

	assert_same(meth, meth.returns("/int"))
	assert_equal("int()", meth.signature)
	assert(meth.arguments.empty?)

	assert_same(meth, meth.argument("arg1", "/std/string", "first argument"))
	assert_equal("int(std::string arg1)", meth.signature(true))
	assert_equal("int(std::string)", meth.signature(false))
	expected_arguments = [["arg1", component.registry.get('std/string'), "first argument"]]
	assert_equal(expected_arguments, meth.arguments)

	meth.argument "arg2", "double", "second argument"
	assert_equal("int(std::string arg1, double arg2)", meth.signature(true))
	assert_equal("int(std::string, double)", meth.signature(false))
	expected_arguments << ["arg2", component.registry.get('double'), "second argument"]
	assert_equal(expected_arguments, meth.arguments)

	meth.returns nil
	assert_equal("void(std::string arg1, double arg2)", meth.signature(true))
	assert_equal("void(std::string, double)", meth.signature(false))
	assert_equal(expected_arguments, meth.arguments)

        create_wc("tasks/method")
	compile_wc(component)
    end

    def test_task_command
	component = Component.new 
	component.name 'test'

	task = component.task_context "task"

	cmd = task.command("Cmd").
	    doc("the command to test")
	assert_equal([cmd], task.self_commands)

	assert_kind_of(Generation::Command, cmd)
	assert_equal("Cmd", cmd.name)
	assert_equal("cmd", cmd.work_method_name)
	assert_equal("isCmdCompleted", cmd.completion_method_name)
	assert_equal("the command to test",  cmd.doc)
	assert(cmd.arguments.empty?)
	assert_equal("()", cmd.work_signature)
	assert_equal("()", cmd.completion_signature)

	assert_same cmd, cmd.work_method_name("another_method_name")
	assert_equal("Cmd", cmd.name)
	assert_equal("another_method_name", cmd.work_method_name)
	assert_equal("isCmdCompleted", cmd.completion_method_name)

	assert_same cmd, cmd.completion_method_name("another_completion_name")
	assert_equal("Cmd", cmd.name)
	assert_equal("another_method_name", cmd.work_method_name)
	assert_equal("another_completion_name", cmd.completion_method_name)

	assert_same cmd, cmd.argument("arg1", "/std/string", "first argument")
	assert_equal("(std::string arg1)", cmd.work_signature)
	assert_equal("(std::string arg1)", cmd.completion_signature)
	assert_equal("(std::string)", cmd.work_signature(false))
	assert_equal("(std::string)", cmd.completion_signature(false))
	expected_arguments = [["arg1", component.registry.get('std/string'), "first argument"]]
	assert_equal(expected_arguments, cmd.arguments)

	cmd.argument "arg2", "double", "second argument"
	assert_equal("(std::string arg1, double arg2)", cmd.work_signature)
	assert_equal("(std::string arg1, double arg2)", cmd.completion_signature)
	assert_equal("(std::string, double)", cmd.work_signature(false))
	assert_equal("(std::string, double)", cmd.completion_signature(false))
	expected_arguments << ["arg2", component.registry.get('double'), "second argument"]
	assert_equal(expected_arguments, cmd.arguments)

	cmd.completion_first_argument
	assert_equal("(std::string arg1)", cmd.completion_signature)
	assert_equal("(std::string)", cmd.completion_signature(false))
	cmd.completion_no_arguments
	assert_equal("()", cmd.completion_signature)
	assert_equal("()", cmd.completion_signature(false))

        create_wc("tasks/command")
	compile_wc(component)
    end

    def test_task_ports_declaration
	component = Component.new 
	component.name 'test'

	task = component.task_context "task"
	read = task.read_port 'r', 'int'
	assert_kind_of(ReadPort, read)
	assert_equal("RTT::ReadPort", read.orocos_class)
	assert_equal("int", read.type.full_name('::', true))

	write = task.write_port 'w', 'int'
	assert_kind_of(WritePort, write)
	assert_equal("RTT::WritePort", write.orocos_class)
	assert_equal("int", write.type.full_name('::', true))

	# read buffer do not have any size
	assert_raises(ArgumentError) { task.read_port 'r', 'int' }
	assert_raises(ArgumentError) { task.read_port 'w', 'int' }
	assert_raises(ArgumentError) { task.write_port 'r', 'int' }
	assert_raises(ArgumentError) { task.write_port 'w', 'int' }

        create_wc("tasks/ports")
	compile_wc(component)
    end
    def test_task_ports_driven
	component = Component.new 
	component.name 'test'

	task  = component.task_context "task"
	read  = task.read_port 'r', 'int'
	write = task.write_port 'w', 'int'

        assert_nothing_raised { task.port_driven 'r' }
        assert_raises(ArgumentError) { task.port_driven 'r1' }
        assert_raises(ArgumentError) { task.port_driven 'w' }
        assert_equal([read], task.event_ports)

        create_wc("tasks/port_driven")
	compile_wc(component)
    end

    def test_default_activity
	component = Component.new 
	component.name 'test'

	task = component.task_context "task"
        assert_raises(ArgumentError) { task.default_activity(:bla) }
        task.default_activity :periodic, 10
        assert_equal([:periodic, 10], task.default_activity)

        deployment = component.static_deployment
        activity = deployment.task "task"
        assert_equal("PeriodicActivity", activity.activity_type)
        assert_equal(10, activity.period)

        create_wc("tasks/default_activity")
	compile_wc(component)
    end
end

