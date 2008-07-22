require 'orocos/generation/test'

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
        component.generate
    end

    def test_task
	component = Component.new
	component.name 'test'

	name = 'task_name'
	doc  = 'task doc'

	task = component.task_context(name)
	assert_raises(ArgumentError) { component.task_context(name) }

	assert_kind_of(Generation::TaskContext, task)
	assert_equal(name, task.name)

	# Check name validation
	assert_raises(ArgumentError) { component.task_context("bla bla") }
	assert_raises(ArgumentError) { component.task_context("bla(bla") }
	assert_raises(ArgumentError) { component.task_context("bla!bla") }
	assert_raises(ArgumentError) { component.task_context("bla/bla") }

	compile_wc(component)
    end

    def test_task_property
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

	compile_wc(component)
    end

    def test_task_method
	component = Component.new
	component.name 'test'

	task = component.task_context("Task")

	meth = task.method("MethodName").
	    doc("the method to test")
	assert_equal([meth], task.methods)

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

	compile_wc(component)
    end

    def test_task_command
	component = Component.new 
	component.name 'test'

	task = component.task_context "task"

	cmd = task.command("Cmd").
	    doc("the command to test")
	assert_equal([cmd], task.commands)

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

	compile_wc(component)
    end

    def test_task_ports
	component = Component.new 
	component.name 'test'

	task = component.task_context "task"
	data_rw = task.data_port 'data_rw', 'int', 'rw'
	assert(data_rw.read_access?)
	assert(data_rw.write_access?)
	assert(data_rw.read_write?)
	assert_equal("DataPort", data_rw.orocos_class)
	assert_equal("int", data_rw.type.full_name('::', true))

	data_w = task.data_port 'data_w', 'int', 'w'
	assert(! data_w.read_access?)
	assert(data_w.write_access?)
	assert(! data_w.read_write?)
	assert_equal("WriteDataPort", data_w.orocos_class)

	data_r = task.data_port 'data_r', 'int', 'r'
	assert(data_r.read_access?)
	assert(! data_r.write_access?)
	assert(! data_r.read_write?)
	assert_equal("ReadDataPort", data_r.orocos_class)

	assert_raises(ArgumentError) { task.data_port 'data_rw', 'int', 'rw' }
	assert_raises(ArgumentError) { task.data_port 'data', 'int', 'bla' }

	assert_raises(ArgumentError) { task.buffer_port 'buffer_r', 'int', 'rw' }
	buffer_rw = task.buffer_port 'buffer_rw', 'int', 10, 'rw'
	assert(buffer_rw.read_access?)
	assert(buffer_rw.write_access?)
	assert(buffer_rw.read_write?)
	assert_equal("BufferPort", buffer_rw.orocos_class)

	assert_raises(ArgumentError) { task.buffer_port 'buffer_r', 'int', 'w' }
	buffer_w = task.buffer_port 'buffer_w', 'int', 10, 'w'
	assert(! buffer_w.read_access?)
	assert(buffer_w.write_access?)
	assert(! buffer_w.read_write?)
	assert_equal("WriteBufferPort", buffer_w.orocos_class)

	# read buffer do not have any size
	assert_raises(ArgumentError) { task.buffer_port 'buffer_r', 'int', 10, 'r' }
	buffer_r = task.buffer_port 'buffer_r', 'int', 'r'
	assert(buffer_r.read_access?)
	assert(! buffer_r.write_access?)
	assert(! buffer_r.read_write?)
	assert_equal("ReadBufferPort", buffer_r.orocos_class)

	compile_wc(component)
    end

    def test_default_activity
	component = Component.new 
	component.name 'test'

	task = component.task_context "task"
        assert_raises(ArgumentError) { task.default_activity(:bla) }
        task.default_activity :period, 10
        assert_equal([:period, 10], task.default_activity)

        deployment = component.static_deployment
        activity = deployment.task "task"
        assert_equal("PeriodicActivity", activity.activity_type)

	compile_wc(component)
    end
end

