require 'orogen/test'

describe Orocos::Spec::TaskContext do
    include Orocos::Generation::Test
    describe "#property" do
        attr_reader :project, :task
        before do
            @task = Orocos::Spec::TaskContext.new(Orocos::Generation::Project.new)
        end

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
    end

    describe "dynamic port support" do
        attr_reader :task, :input_port, :output_port
        before do
            @task = Orocos::Spec::TaskContext.new(Orocos::Generation::Project.new)
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
                @task = Orocos::Spec::TaskContext.new(parent_task.project)
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
            @task = Orocos::Spec::TaskContext.new(Orocos::Generation::Project.new, "test::Task")
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
