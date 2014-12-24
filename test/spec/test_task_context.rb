require 'orogen/test'

describe OroGen::Spec::TaskContext do
    attr_reader :project, :loader, :task
    before do
        @loader = OroGen::Loaders::Files.new
        OroGen::Loaders::RTT.setup_loader(loader)
        @project = OroGen::Spec::Project.new(loader)
        project.name 'base'
        @task = project.task_context 'Task'
    end

    describe "#task_model_from_name" do
        attr_reader :dependency_task
        before do
            # Create another project
            dependency = OroGen::Spec::Project.new(loader)
            dependency.name 'dependency'
            @dependency_task = dependency.task_context 'Foo'
            project.using_task_library dependency
        end

        it "should resolve self tasks without the project's name" do
            assert_equal(task, project.task_model_from_name("Task"))
        end
        it "should resolve tasks by full name" do
            assert_equal(task, project.task_model_from_name("base::Task"))
        end
        it "should raise ArgumentError if the task does not exist" do
            assert_raises(OroGen::TaskModelNotFound) { project.task_model_from_name("Bla") }
        end
        it "should raise ArgumentError if a task from a dependency is required as a relative name" do
            assert_raises(OroGen::TaskModelNotFound) { project.task_model_from_name("Foo") }
        end
        it "should resolve tasks from a dependency by full name" do
            assert_equal dependency_task, project.task_model_from_name("dependency::Foo")
        end
    end

    describe "#find_task_context" do
        attr_reader :dependency_task
        before do
            # Create another project
            dependency = OroGen::Spec::Project.new(loader)
            dependency.name 'dependency'
            @dependency_task = dependency.task_context 'Foo'
            project.using_task_library dependency
        end

        it "should resolve self tasks without the project's name" do
            assert_equal(task, project.find_task_context("Task"))
        end
        it "should resolve tasks by full name" do
            assert_equal(task, project.find_task_context("base::Task"))
        end
        it "should raise ArgumentError if the task does not exist" do
            assert_raises(ArgumentError) { project.find_task_context("Bla") }
        end
        it "should raise ArgumentError if a task from a dependency is required as a relative name" do
            assert_raises(ArgumentError) { project.find_task_context("Foo") }
        end
        it "should resolve tasks from a dependency by full name" do
            assert_equal dependency_task, project.find_task_context("dependency::Foo")
        end
    end

    describe "#property" do
        it "should not allow for duplicate names" do
            task.property("bla", "/double")
            assert_raises(ArgumentError) { task.property("bla", "/double") }
        end

        it "should validate property names" do
            assert_raises(ArgumentError) { task.property("bla bla", "/double") }
            assert_raises(ArgumentError) { task.property("bla(bla", "/double") }
            assert_raises(ArgumentError) { task.property("bla!bla", "/double") }
            assert_raises(ArgumentError) { task.property("bla/bla", "/double") }
        end

        it "should register the property in #self_properties" do
            property = task.property("p", '/double')
            assert_equal [property], task.self_properties
        end
        
        it "should properly set the property attributes" do
            property = task.property("p", '/double').doc("property")
            assert_kind_of(OroGen::Spec::Property, property)
            assert_equal("p", property.name)
            assert_equal('/double', property.type.full_name)
            assert_equal("property",  property.doc)
        end
    end

    describe "#operation" do
        it "registers the operation in self_operations" do
            meth = task.operation("MethodName")
            assert_kind_of OroGen::Spec::Operation, meth
            assert_equal [meth], task.self_operations
        end
        it "sets the operation name" do
            meth = task.operation("method_name")
            assert_equal 'method_name', meth.name
        end
        it "sets a default of 'returning nothing'" do
            meth = task.operation("method_name")
            assert_equal([nil, 'void', ''], meth.return_type)
        end
    end

    describe "#input_port" do
        it "raises ArgumentError if a port with the same name already exists" do
            task.input_port 'i', 'int'
            task.output_port 'o', 'int'
            assert_raises(ArgumentError) { task.input_port 'i', 'int' }
            assert_raises(ArgumentError) { task.input_port 'o', 'int' }
        end
    end

    describe "#output_port" do
        it "raises ArgumentError if a port with the same name already exists" do
            task.input_port 'i', 'int'
            task.output_port 'o', 'int'
            assert_raises(ArgumentError) { task.output_port 'i', 'int' }
            assert_raises(ArgumentError) { task.output_port 'o', 'int' }
        end
    end

    describe "#port_driven" do
        it "can handle a port by name" do
            p = task.input_port 'r', 'int'
            task.port_driven 'r'
            assert_equal [p], task.all_event_ports
        end
        it "raises ArgumentError if given a non-existing port" do
            assert_raises(ArgumentError) { task.port_driven 'r' }
        end
        it "raises ArgumentError if given an output port" do
            task.output_port 'w', 'int'
            assert_raises(ArgumentError) { task.port_driven 'w' }
        end
    end

    describe "#state_kind" do
        it "returns the type of a given state" do
            task.runtime_states 'TEST'
            assert_equal :runtime, task.state_kind('TEST')
        end
        it "returns nil if the state is not known" do
            assert_equal nil, task.state_kind('test')
        end
    end

    describe "#define_state" do
        it "raises ArgumentError if the state type is not known" do
            assert_raises(ArgumentError) { task.define_state 'TEST', :bla }
        end
        it "raises ArgumentError if the state is already defined but with a different kind" do
            task.define_state 'TEST', :runtime
            assert_raises(ArgumentError) { task.define_state 'TEST', :error }
        end
        it "does nothing if the state is already defined with the same type" do
            task.define_state 'TEST', :runtime
            task.define_state 'TEST', :runtime
        end
    end

    OroGen::Spec::TaskContext::STATE_TYPES.each do |type|
        next if type == :toplevel
        it "can define and enumerate #{type} states" do
            task.send("#{type}_states", "STATE0", "STATE1")
            assert_equal ["STATE0", "STATE1"], task.send("each_#{type}_state").to_a
        end
        it "can enumerate states from the superclass" do
            superclass = project.task_context "Base"
            superclass.send("#{type}_states", "STATE0")
            task.subclasses superclass
            task.send("#{type}_states", "STATE1")
            assert_equal ["STATE0", "STATE1"], task.send("each_#{type}_state").to_a
        end
    end

    describe "dynamic port support" do
        attr_reader :input_port, :output_port
        before do
            @input_port = task.dynamic_input_port(/r$/, "/int")
            @output_port = task.dynamic_output_port(/w$/, "/int")
        end

        describe "#each_dynamic_input_port" do
            it "should list the available dynamic input ports" do
                assert_equal [input_port], task.each_dynamic_input_port.to_a
            end
        end
        describe "#find_dynamic_input_ports" do
            it "can find matching names in the set of ports returned by #each_dynamic_input_port" do
                flexmock(task).should_receive(:each_dynamic_input_port).twice.and_return([input_port])
                assert_equal [input_port], task.find_dynamic_input_ports("blar", nil)
                assert_equal [], task.find_dynamic_input_ports("blaw", nil)
            end
            it "can find matching name and type in the set of ports returned by #each_dynamic_input_port" do
                flexmock(task).should_receive(:each_dynamic_input_port).twice.and_return([input_port])
                assert_equal [input_port], task.find_dynamic_input_ports("blar", "/int")
                assert_equal [], task.find_dynamic_input_ports("blar", "/double")
            end
        end
        describe "#has_dynamic_input_port?" do
            it "returns false if #find_dynamic_input_ports returns an empty set" do
                flexmock(task).should_receive(:find_dynamic_input_ports).with('name', '/type').and_return([])
                assert !task.has_dynamic_input_port?('name', '/type')
            end
            it "returns true if #find_dynamic_input_ports returns a non-empty set" do
                flexmock(task).should_receive(:find_dynamic_input_ports).with('name', '/type').and_return([true])
                assert task.has_dynamic_input_port?('name', '/type')
            end
        end
        describe "#each_dynamic_output_port" do
            it "should list the available dynamic input ports" do
                assert_equal [output_port], task.each_dynamic_output_port.to_a
            end
        end
        describe "#find_dynamic_output_ports" do
            it "can find matching names in the set of ports returned by #each_dynamic_output_port" do
                flexmock(task).should_receive(:each_dynamic_output_port).twice.and_return([output_port])
                assert_equal [output_port], task.find_dynamic_output_ports("blaw", nil)
                assert_equal [], task.find_dynamic_output_ports("blar", nil)
            end
            it "can find matching name and type in the set of ports returned by #each_dynamic_output_port" do
                flexmock(task).should_receive(:each_dynamic_output_port).twice.and_return([output_port])
                assert_equal [output_port], task.find_dynamic_output_ports("blaw", "/int")
                assert_equal [], task.find_dynamic_output_ports("blaw", "/double")
            end
        end
        describe "#has_dynamic_output_port?" do
            it "returns false if #find_dynamic_output_ports returns an empty set" do
                flexmock(task).should_receive(:find_dynamic_output_ports).with('name', '/type').and_return([])
                assert !task.has_dynamic_output_port?('name', '/type')
            end
            it "returns true if #find_dynamic_output_ports returns a non-empty set" do
                flexmock(task).should_receive(:find_dynamic_output_ports).with('name', '/type').and_return([true])
                assert task.has_dynamic_output_port?('name', '/type')
            end
        end

        describe "in subclasses" do
            attr_reader :parent_task
            before do
                @parent_task = task
                @task = OroGen::Spec::TaskContext.new(parent_task.project)
                task.subclasses parent_task
            end

            describe "#each_dynamic_input_port" do
                it "should list the dynamic input ports from its parent" do
                    assert_equal [input_port], task.each_dynamic_input_port.to_a
                end
            end
            describe "#each_dynamic_output_port" do
                it "should list the dynamic output ports from its parent" do
                    assert_equal [output_port], task.each_dynamic_output_port.to_a
                end
            end
        end
    end

    describe "#to_h" do
        attr_reader :task
        before do
            @task = OroGen::Spec::TaskContext.new(create_dummy_project, "test::Task")
        end
        it "marshals the model in hash form" do
            task.runtime_states :TEST
            port      = task.input_port('in_p', '/int')
            property  = task.property('p', '/double')
            attribute = task.attribute('a', '/double')
            operation = task.operation('op')
            h = task.to_h
            assert_equal task.name, h[:name]
            assert_equal task.superclass.name, h[:superclass]
            assert_equal [port.to_h, task.find_port('state').to_h], h[:ports]
            assert_equal [property.to_h], h[:properties]
            assert_equal [attribute.to_h], h[:attributes]
            assert_equal [operation.to_h], h[:operations]

            expected_states = task.superclass.each_state.to_a + [['TEST', :runtime]]
            assert_equal expected_states, h[:states]
        end
    end
end
