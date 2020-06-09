require 'orogen/test'

describe OroGen::Spec::Project do
    attr_reader :project, :loader
    before do
        @loader = OroGen::Loaders::Files.new
        OroGen::Loaders::RTT.setup_loader(loader)
        @project = OroGen::Spec::Project.new(loader)
        project.name 'base'
    end

    describe "#task_context" do
        it "should set the task name to its full name" do
            task = project.task_context 'Task'
            assert_equal 'base::Task', task.name
        end
        it "should reject task whose name is identical to the project" do
            assert_raises(ArgumentError) { project.task_context 'base' }
        end
        it "should validate task names" do
            assert_raises(ArgumentError) { project.task_context '&' }
            assert_raises(ArgumentError) { project.task_context("bla bla") }
            assert_raises(ArgumentError) { project.task_context("bla(bla") }
            assert_raises(ArgumentError) { project.task_context("bla!bla") }
            assert_raises(ArgumentError) { project.task_context("bla/bla") }
        end
        it "should raise ArgumentError if the task already exists" do
            project.task_context 'Task'
            assert_raises(ArgumentError) { project.task_context 'Task' }
        end
    end
end

