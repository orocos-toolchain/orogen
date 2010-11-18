$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orogen/test'

class TC_GenerationTasks < Test::Unit::TestCase
    include Orocos::Generation::Test

    # Orogen should refuse to create a task context which has the same name than
    # one namespace of the type registry (that would not compile)
    def test_task_name_should_not_clash_with_namespace_name
        component = Component.new
        component.typekit(true).load File.join(TEST_DATA_DIR, 'modules', 'typekit_simple', 'simple.h')

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

	meth = task.operation("MethodName").
	    doc("the method to test")

        assert_raises(Orocos::Generation::ConfigError) { meth.argument("a", "short") }
        assert_raises(Orocos::Generation::ConfigError) { meth.argument("a", "unsigned short") }
        assert_raises(Orocos::Generation::ConfigError) { meth.argument("a", "char") }
        assert_raises(Orocos::Generation::ConfigError) { meth.argument("a", "unsigned char") }
        assert_raises(Orocos::Generation::ConfigError) { meth.argument("a", "long long") }
        assert_raises(Orocos::Generation::ConfigError) { meth.argument("a", "unsigned long long") }
        assert_raises(Orocos::Generation::ConfigError) { meth.argument("a", "float") }
    end

    def test_task_operation
	component = Component.new
	component.name 'test'

	task = component.task_context("Task")

	meth = task.operation("MethodName").
	    doc("the method to test")
	assert_equal([meth], task.self_operations)

	assert_kind_of(Generation::Operation, meth)
	assert_equal("MethodName", meth.name)
	assert_equal("methodName", meth.method_name)
	assert_equal([nil, 'void'], meth.return_type)
	assert_equal("the method to test",  meth.doc)
	assert(meth.arguments.empty?)
	assert_equal("void methodName()", meth.signature)
	assert_equal("void()", meth.signature(false))
	assert_equal("void foobar()", meth.signature { "foobar" })

	meth.method_name "another_method_name"
	assert_equal("MethodName", meth.name)
	assert_equal("another_method_name", meth.method_name)

	assert_same(meth, meth.returns("/int"))
	assert_equal("boost::int32_t another_method_name()", meth.signature)
	assert(meth.arguments.empty?)

	assert_same(meth, meth.argument("arg1", "/std/string", "first argument"))
	assert_equal("boost::int32_t another_method_name(::std::string const & arg1)", meth.signature(true))
	assert_equal("boost::int32_t(::std::string const &)", meth.signature(false))
	expected_arguments = [["arg1", component.registry.get('std/string'), "first argument"]]
	assert_equal(expected_arguments, meth.arguments)

	meth.argument "arg2", "double", "second argument"
	assert_equal("boost::int32_t another_method_name(::std::string const & arg1, double arg2)", meth.signature(true))
	assert_equal("boost::int32_t(::std::string const &, double)", meth.signature(false))
	expected_arguments << ["arg2", component.registry.get('double'), "second argument"]
	assert_equal(expected_arguments, meth.arguments)

	meth.returns nil
	assert_equal("void another_method_name(::std::string const & arg1, double arg2)", meth.signature(true))
	assert_equal("void(::std::string const &, double)", meth.signature(false))
	assert_equal(expected_arguments, meth.arguments)

        meth.argument "arg3", "double", "third argument"
        meth.argument "arg4", "double", "fourth argument"
        # Should raise if there is more than 4 arguments
        assert_raises(ArgumentError) { meth.argument "arg5", "double", "fifth argument" }

        task.operation(:symbol_name).
            doc "this method's name is a symbol"
        task.operation(:in_caller_thread).
            runs_in_caller_thread.
            doc "this method runs in caller thread"

        create_wc("tasks/method")
	compile_wc(component)
    end

    def test_task_ports_declaration
	component = Component.new 
	component.name 'test'

	task = component.task_context "task"
	read = task.input_port 'r', 'int'
	assert_kind_of(InputPort, read)
	assert_equal("RTT::InputPort", read.orocos_class)
	assert_equal("boost::int32_t", read.type.cxx_name)

	write = task.output_port 'w', 'int'
	assert_kind_of(OutputPort, write)
	assert_equal("RTT::OutputPort", write.orocos_class)
	assert_equal("boost::int32_t", write.type.cxx_name)

	# read buffer do not have any size
	assert_raises(ArgumentError) { task.input_port 'r', 'int' }
	assert_raises(ArgumentError) { task.input_port 'w', 'int' }
	assert_raises(ArgumentError) { task.output_port 'r', 'int' }
	assert_raises(ArgumentError) { task.output_port 'w', 'int' }

        create_wc("tasks/ports")
	compile_wc(component)
    end
    def test_task_port_static
	component = Component.new 
	component.name 'test'

	task = component.task_context "task"

	read = task.input_port 'r', 'int'
	write = task.output_port 'w', 'int'
        [read, write].each do |p|
            assert(!p.static?)
            p.static
            assert(p.static?)
            p.dynamic
            assert(!p.static?)
        end
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
        activity = deployment.task "task", "task"
        activity_def = activity.activity_type
        assert_equal("Activity", activity_def.name)
        assert_equal(10, activity.period)

        create_wc("tasks/default_activity")
	compile_wc(component)
    end

    def test_needs_configuration
        build_test_component('modules/with_configuration', [])
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

        assert_equal [
            ["INIT", :toplevel],                                                                                            
            ["PRE_OPERATIONAL", :toplevel],                                                                                  
            ["FATAL_ERROR", :toplevel],                                                                                      
            ["EXCEPTION", :toplevel],                                                                                      
            ["STOPPED", :toplevel],                                                                                          
            ["RUNNING", :toplevel],                                                                                          
            ["RUNTIME_ERROR", :toplevel],
            ['STATE1', :runtime], ['STATE2', :error], ['STATE3', :runtime], ['STATE4', :error]], task.each_state.to_a
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

    def test_state_type_definitions(*transports)
        component = build_test_component('modules/extended_states', transports)
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
end

