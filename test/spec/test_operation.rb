require 'orogen/test'

describe OroGen::Spec::Operation do
    attr_reader :project, :loader, :task_model, :operation
    before do
        @loader = OroGen::Loaders::Files.new
        OroGen::Loaders::RTT.setup_loader(loader)
        @project = OroGen::Spec::Project.new(loader)
        project.name 'base'
        @task_model = project.task_context 'Task'
        @operation = task_model.operation 'test'
    end

    describe "#doc" do
        it "sets the documentation" do
            doc = "test documentation"
            operation.doc(doc)
            assert_equal doc, operation.doc
        end
    end

    describe "#to_h" do
        attr_reader :task, :op
        before do
            @task = OroGen::Spec::TaskContext.new(project)
            @op = task.operation('op')
        end

        it "marshals the name" do
            assert_equal 'op', op.to_h[:name]
        end
        it "marshals empty documentation as an empty string" do
            op.doc ""
            assert_equal "", op.to_h[:doc]
        end
        it "marshals the documentation" do
            op.doc "with documentation"
            assert_equal "with documentation", op.to_h[:doc]
        end
        it "does not add a return field if the operation returns nothing" do
            assert !op.to_h.has_key?(:returns)
        end
        it "marshals the typelib return type if the operation returns a value" do
            op.returns("/double")
            double_t = task.project.find_type('/double')
            assert_equal double_t.to_h,
                op.to_h[:returns][:type]
        end
        it "sets an empty return type documentation if none is specified" do
            op.returns("/double")
            assert_equal "", op.to_h[:returns][:doc]
        end
        it "sets the return type documentation field to the provided documentation" do
            op.returns("/double", "return type description")
            assert_equal "return type description", op.to_h[:returns][:doc]
        end
        it "sets the arguments field to an empty array if there are none" do
            assert_equal [], op.to_h[:arguments]
        end
        it "marshals its arguments" do
            op.argument('arg', '/double')
            double_t = task.project.find_type('/double')
            assert_equal [Hash[name: 'arg', type: double_t.to_h, doc: ""]], op.to_h[:arguments]
        end
        it "marshals its arguments documentation" do
            op.argument('arg', '/double', 'arg documentation')
            double_t = task.project.find_type('/double')
            assert_equal [Hash[name: 'arg', type: double_t.to_h, doc: "arg documentation"]], op.to_h[:arguments]
        end
    end

    describe "#find_interface_type" do
        attr_reader :op, :project
        before do
            task = create_dummy_project.task_context "Task"
            @project = project
            @op = OroGen::Spec::Operation.new(task, 'op')
        end

        it "should strip the qualifiers to resolve the type" do
            type, _ = op.find_interface_type('double const&')
            assert_equal type, op.task.project.find_type('/double')
        end
        it "should replace typelib typenames by C++ typenames in the signature" do
            _, cxx_signature = op.find_interface_type('/double const&')
            assert_equal cxx_signature, 'double const&'
        end
        it "should accept plain Typelib type objects" do
            double_t = project.find_type('/double')
            type, cxx_signature = op.find_interface_type(double_t)
            assert_equal type, double_t
            assert_equal cxx_signature, 'double'
        end
    end
end

