require 'orogen/test'

describe OroGen::Spec::TaskContext do
    attr_reader :project
    before do
        loader = OroGen::Loaders::Files.new
        OroGen::Loaders::RTT.setup_loader(loader)
        @project = OroGen::Spec::Project.new(loader)
    end

    describe "#property" do
        attr_reader :task
        before do
            @task = OroGen::Spec::TaskContext.new(project)
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
            @task = OroGen::Spec::TaskContext.new(project)
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
end
