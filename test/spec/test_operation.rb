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
end
