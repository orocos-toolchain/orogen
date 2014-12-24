require 'orogen/gen/test'

class TC_GenerationTasks < Minitest::Test
    attr_reader :task, :project
    def setup
        @project = OroGen::Gen::RTT_CPP::Project.new
        project.name 'test'
        @task = project.task_context 'Task'
        super
    end

    # Orogen should refuse to create a task context which has the same name than
    # one namespace of the type registry (that would not compile)
    def test_task_name_should_not_clash_with_namespace_name
        project.deffile = File.join(path_to_data, 'modules', 'typekit_simple', 'simple.orogen')
        project.typekit(true).load File.join(path_to_data, 'modules', 'typekit_simple', 'simple.h')
        assert_raises(ArgumentError) { project.task_context("Test") {} }
    end

    def test_generation_of_an_empty_task
        create_wc("tasks/task_context")
        compile_wc(project)
    end

    def test_generation_of_properties
        %w{/double /std/vector</double>}.each_with_index do |typename, i|
            task.property("p#{i}", typename).doc("property #{i}")
        end

        create_wc("tasks/property")
	compile_wc(project)
    end

    def test_operation_sets_the_method_name_to_the_op_name_with_a_lower_first_character
	meth = task.operation("Method_name")
	assert_equal("method_name", meth.method_name)
    end

    def test_operation_empty_signature
	meth = task.operation("methodName")
	assert(meth.arguments.empty?)
	assert_equal("void methodName()", meth.signature)
	assert_equal("void()", meth.signature(false))
	assert_equal("void foobar()", meth.signature { "foobar" })
    end

    def test_operation_can_override_the_method_name
	meth = task.operation("method_name")
        meth.method_name "method_name"
	assert_equal("void method_name()", meth.signature)
    end

    def test_operation_returns
	meth = task.operation("methodName")
	assert_same(meth, meth.returns("/int"))
	assert_equal("boost::int32_t methodName()", meth.signature)
        meth.returns nil
	assert_equal("void methodName()", meth.signature)
    end

    def test_operation_argument_add_one_argument
	meth = task.operation("methodName")
        ret = meth.argument("arg1", "/std/string", "first argument").
            returns('int32_t')
	assert_same(meth, ret)
	assert_equal("boost::int32_t methodName(::std::string const & arg1)", meth.signature(true))
	assert_equal("boost::int32_t(::std::string const &)", meth.signature(false))
	expected_arguments = [
            ["arg1", project.registry.get('std/string'), "first argument", "::std::string"]
        ]
	assert_equal(expected_arguments, meth.arguments)
    end

    def test_operation_argument_add_two_arguments
	meth = task.operation("method_name")
	meth.argument("arg1", "/std/string", "first argument")
	meth.argument "arg2", "double", "second argument"
	assert_equal("void method_name(::std::string const & arg1, double arg2)", meth.signature(true))
	assert_equal("void(::std::string const &, double)", meth.signature(false))
	expected_arguments = [
            ["arg1", project.registry.get('std/string'), "first argument", "::std::string"],
            ["arg2", project.registry.get('double'), "second argument", "double"]
        ]
	assert_equal(expected_arguments, meth.arguments)
    end

    def test_operation_argument_raises_ArgumentError_if_more_than_RTT_OPERATIONS_ARGUMENT_COUNT_LIMIT_arguments_are_set
	meth = task.operation("method_name")
        OroGen::Spec::Operation::RTT_ARGUMENT_COUNT_LIMIT.times do |i|
            meth.argument "arg#{i}", "double"
        end
        assert_raises(ArgumentError) { meth.argument "argX", "double" }
    end

    def test_generation_of_operations
	task.operation("MethodName").
            argument("arg1", "/std/string", "first argument").
            argument("arg2", "double", "second argument").
            argument("arg3", "double", "third argument").
            argument("arg4", "double", "fourth argument").
            returns('/std/string')

        task.operation(:symbol_name).
            doc "this method's name is a symbol"
        task.operation(:in_caller_thread).
            runs_in_caller_thread.
            doc "this method runs in caller thread"

        create_wc("tasks/method")
	compile_wc(project)
    end

    def test_input_port_sets_the_orocos_class_properly
	p = task.input_port 'r', 'int'
	assert_equal("RTT::InputPort", p.orocos_class)
    end

    def test_output_port_sets_the_orocos_class_properly
	p = task.output_port 'r', 'int'
	assert_equal("RTT::OutputPort", p.orocos_class)
    end

    def test_generation_of_input_and_output_ports
	task.input_port 'r', 'int'
	task.output_port 'w', 'int'

        create_wc("tasks/ports")
	compile_wc(project)
    end

    def test_task_ports_driven
	task.input_port 'r', 'int'
        task.port_driven 'r'

        create_wc("tasks/port_driven")
	compile_wc(project)
    end

    def test_it_can_generate_tasks_with_a_default_activity
        task.default_activity :periodic, 10
        deployment = project.deployment 'test'
        deployment.task 'test', task
        create_wc("tasks/default_activity")
	compile_wc(project)
    end

    def test_needs_configuration
        build_test_project('modules/with_configuration', [])
        install
    end

    def test_it_can_generate_tasks_with_dynamic_ports
        task.dynamic_input_port(/r$/, "/int")
        task.dynamic_output_port(/w$/, "/double")
        create_wc("tasks/dynamic_ports")
	compile_wc(project)
    end

    def test_state_type_definitions(*transports)
        project = build_test_project('modules/extended_states', transports)
        install

        parent = project.find_task_context "Parent"
        child  = project.find_task_context "Child"

        parent_states = project.find_type "/extstate/Parent_STATES"
        child_states  = project.find_type "/extstate/Child_STATES"

        parent_values = parent_states.keys
        child_values  = child_states.keys
        assert_equal parent_values['Parent_STATE1'], child_values['Child_STATE1']
        assert_equal parent_values['Parent_STATE2'], child_values['Child_STATE2']
    end

    def test_default_values
        project.deffile = File.join(path_to_wc_root, 'test.orogen')
        Tempfile.open("orogen_test_default_values") do |io|
            io.puts <<-EOCODE
enum TestEnum {
    VALUE0 = 0,
    VALUE1 = 1
};

struct AStruct {
    int bla;
};
            EOCODE
            io.flush
            project.import_types_from io.path
            project.typekit.perform_pending_loads
        end

        assert_raises(ArgumentError) { task.property('str', '/std/string', 3) }
        task.property('str', '/std/string', "3")
        
        assert_raises(ArgumentError) { task.property('num', '/double', "3") }
        task.property('num', '/double', 3)
        
        assert_raises(ArgumentError) { task.property('enum', '/TestEnum', "3") }
        assert_raises(ArgumentError) { task.property('enum', '/TestEnum', 3) }
        assert_raises(ArgumentError) { task.property('enum', '/TestEnum', "VALUE10") }
        task.property('enum', '/TestEnum', :VALUE0)
        task.property('enum1', '/TestEnum', 'VALUE0')

        assert_raises(ArgumentError) { task.property('struct', '/AStruct', "3") }
        assert_raises(ArgumentError) { task.property('struct', '/AStruct', :bla => 3) }

        assert_raises(ArgumentError) { task.property('bool', '/int', "VALUE10") }

        assert_raises(ArgumentError) { task.property('bool', '/bool', "VALUE10") }
        task.property('bool', '/bool', true)
        task.property('bool1', '/bool', false)
    end
end

