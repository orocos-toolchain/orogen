$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orogen/test'

class TC_GenerationTasks < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

    # Orogen should refuse to create a task context which has the same name than
    # one namespace of the type registry (that would not compile)
    def test_task_name_should_not_clash_with_namespace_name
        component = Component.new
        component.toolkit do
            load File.join(TEST_DATA_DIR, 'modules', 'toolkit_simple', 'simple.h')
        end

        assert_raises(ArgumentError) { component.task_context("Test") {} }
    end

    # Test the definition of a basic task context
    def test_empty_task
	component = Component.new
	component.name 'test'
	task = component.task_context("Task")
        create_wc("tasks/task_context")
        compile_wc(component)
    end

    def test_task_name
	component = Component.new
	component.name 'test'
	task = component.task_context("Task")
        assert_equal("test::Task", task.name)
    end

    def test_task_name_validation
	component = Component.new
	assert_raises(ArgumentError) { component.task_context("bla bla") }
	assert_raises(ArgumentError) { component.task_context("bla(bla") }
	assert_raises(ArgumentError) { component.task_context("bla!bla") }
	assert_raises(ArgumentError) { component.task_context("bla/bla") }
    end
    def test_no_duplicate_task_names
	component = Component.new
        component.name "name"
	component.task_context("task_name")
	assert_raises(ArgumentError) { component.task_context("task_name") }
    end

    def test_property
	component = Component.new
	component.name 'test'

	task = component.task_context("task_name")
        %w{/double /std/vector</double>}.each_with_index do |typename, i|
            property = task.property("p#{i}", typename).doc("property #{i}")
            assert_kind_of(Generation::Property, property)
            assert_equal("p#{i}", property.name)
            assert_equal(typename, property.type.full_name)
            assert_equal("property #{i}",  property.doc)
        end

        create_wc("tasks/property")
	compile_wc(component)
    end

    def test_no_duplicate_property_name
	component = Component.new
        component.name "project"
        task = component.task_context "Task"
	task.property("bla", "/double")
	assert_raises(ArgumentError) { task.property("bla", "/double") }
    end
    def test_property_name_validation
	component = Component.new
        component.name "project"
        task = component.task_context "Task"
	assert_raises(ArgumentError) { task.property("bla bla", "/double") }
	assert_raises(ArgumentError) { task.property("bla(bla", "/double") }
	assert_raises(ArgumentError) { task.property("bla!bla", "/double") }
	assert_raises(ArgumentError) { task.property("bla/bla", "/double") }
    end

    def test_validate_toplevel_types
	component = Component.new
	component.name 'test'

	task = component.task_context("Task")

	meth = task.method("MethodName").
	    doc("the method to test")

        assert_raises(ArgumentError) { meth.argument("a", "short") }
        assert_raises(ArgumentError) { meth.argument("a", "unsigned short") }
        assert_raises(ArgumentError) { meth.argument("a", "char") }
        assert_raises(ArgumentError) { meth.argument("a", "unsigned char") }
        assert_raises(ArgumentError) { meth.argument("a", "long long") }
        assert_raises(ArgumentError) { meth.argument("a", "unsigned long long") }
        assert_raises(ArgumentError) { meth.argument("a", "float") }
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

        meth.argument "arg3", "double", "third argument"
        meth.argument "arg4", "double", "fourth argument"
        # Should raise if there is more than 4 arguments
        assert_raises(ArgumentError) { meth.argument "arg5", "double", "fifth argument" }

        task.method(:symbol_name).
            doc "this method's name is a symbol"

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

        cmd.argument "arg3", "double", "third argument"
        cmd.argument "arg4", "double", "fourth argument"
        # Should raise if there is more than 4 arguments
        assert_raises(ArgumentError) { cmd.argument "arg5", "double", "fifth argument" }

        create_wc("tasks/command")
	compile_wc(component)
    end

    def test_task_ports_declaration
	component = Component.new 
	component.name 'test'

	task = component.task_context "task"
	read = task.input_port 'r', 'int'
	assert_kind_of(InputPort, read)
	assert_equal("RTT::InputPort", read.orocos_class)
	assert_equal("int", read.type.full_name('::', true))

	write = task.output_port 'w', 'int'
	assert_kind_of(OutputPort, write)
	assert_equal("RTT::OutputPort", write.orocos_class)
	assert_equal("int", write.type.full_name('::', true))

	# read buffer do not have any size
	assert_raises(ArgumentError) { task.input_port 'r', 'int' }
	assert_raises(ArgumentError) { task.input_port 'w', 'int' }
	assert_raises(ArgumentError) { task.output_port 'r', 'int' }
	assert_raises(ArgumentError) { task.output_port 'w', 'int' }

        create_wc("tasks/ports")
	compile_wc(component)
    end
    def test_task_ports_driven
	component = Component.new 
	component.name 'test'

	task  = component.task_context "task"
	read  = task.input_port 'r', 'int'
	write = task.output_port 'w', 'int'

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

    def test_needs_configuration
        build_test_component('modules/with_configuration', false)
        install
    end

    def test_dynamic_ports
	component = Component.new 
	component.name 'test'

	task = component.task_context "task"
        task.dynamic_input_port(/r$/, "/int")
        task.dynamic_output_port(/w$/, "/double")

        assert task.dynamic_input_port?("hgkur", "/int")
        assert !task.dynamic_input_port?("hgkur", "/double")
        assert !task.dynamic_input_port?("hgkuw", "/int")

        assert task.dynamic_output_port?("hgkuw", "/double")
        assert !task.dynamic_output_port?("hgkur", "/double")
        assert !task.dynamic_output_port?("hgkuw", "/int")

        create_wc("tasks/dynamic_ports")
	compile_wc(component)
    end

    def test_state_definitions
        component = Component.new
        component.name "test"

        task = component.task_context "Task"
        task.runtime_states "STATE1", "STATE3"
        assert_equal ["STATE1", "STATE3"], task.each_runtime_state.to_a

        task.error_states 'STATE2', 'STATE4'
        assert_equal ["STATE1", "STATE3"], task.each_runtime_state.to_a
        assert_equal ["STATE2", "STATE4"], task.each_error_state.to_a

        assert_equal [['STATE1', :runtime], ['STATE2', :error], ['STATE3', :runtime], ['STATE4', :error]], task.each_state.to_a
    end

    def test_state_avoid_duplicates
        component = Component.new
        component.name "test"

        task = component.task_context "Task"
        task.runtime_states "STATE1"
        task.error_states "STATE2"

        assert_raises(ArgumentError) { task.error_states("STATE1") }
        task.runtime_states("STATE1")
        task.error_states("STATE2")
        assert_raises(ArgumentError) { task.runtime_states("STATE2") }
    end

    def test_state_inheritance
        component = Component.new
        component.name "test"


        parent = component.task_context "Parent"
        parent.runtime_states "STATE1"
        parent.error_states "STATE2"
        parent_states = parent.each_state.to_a
        rtt = parent.superclass
        rtt_states = rtt.each_state.to_a
        assert_equal (rtt_states + [["STATE1", :runtime], ["STATE2", :error]]), parent_states

        child  = component.task_context "Child"
        child.subclasses parent
        child_states = child.each_state.to_a
        assert_equal parent_states, child_states

        assert_raises(ArgumentError) { child.error_states("STATE1") }
        child.runtime_states("STATE1")
        child.error_states("STATE2")
        assert_raises(ArgumentError) { child.runtime_states("STATE2") }
    end

    def test_state_type_definitions(with_corba = true)
        component = build_test_component('modules/extended_states', with_corba)
        install

        parent = component.find_task_context "Parent"
        child  = component.find_task_context "Child"

        parent_states = component.find_type "/extstate/Parent_STATES"
        child_states  = component.find_type "/extstate/Child_STATES"

        parent_values = parent_states.keys
        child_values  = child_states.keys
        assert_equal parent_values['Parent_STATE1'], child_values['Child_STATE1']
        assert_equal parent_values['Parent_STATE2'], child_values['Child_STATE2']
    end
    def test_state_type_definitions_without_corba
        test_state_type_definitions(false)
    end
end

