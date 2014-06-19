require 'orogen/test'

describe OroGen::Spec::Port do
    attr_reader :project, :loader, :task, :port
    before do
        @loader = OroGen::Loaders::Files.new
        OroGen::Loaders::RTT.setup_loader(loader)
        @project = OroGen::Spec::Project.new(loader)
        project.name 'base'
        @task = project.task_context 'Task'
        @port = OroGen::Spec::Port.new(task, 'test', '/double')
    end

    describe "static or dynamic connection" do
        it "is dynamic by default" do
            assert !port.static?
        end
        it "becomes static if #static is called" do
            port.static
            assert port.static?
        end
        it "becomes dynamic again if #dynamic is called" do
            port.static
            port.dynamic
            assert !port.static?
        end
    end
end

describe OroGen::Spec::OutputPort do
end
