require 'orogen/test'
describe Orocos::Generation::Operation do
    include Orocos::Generation::Test
    it "should accept ro_ptr arguments (#13)" do
        project = Orocos::Generation::Project.new
        project.name "test"
        project.task_context "Task" do
            operation('test').
                arg('arg', ro_ptr('/std/vector</double>'))
        end
        create_wc 'operations_ro_ptr_argument'
        compile_wc(project)
    end
end

