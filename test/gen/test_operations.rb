require 'orogen/gen/test'

describe OroGen::Gen::RTT_CPP::Operation do
    it "should accept ro_ptr arguments (#13)" do
        project = OroGen::Gen::RTT_CPP::Project.new
        project.name "test"
        project.task_context "Task" do
            operation('test').
                arg('arg', ro_ptr('/std/vector</double>'))
        end
        create_wc 'operations_ro_ptr_argument'
        compile_wc(project)
    end
end

