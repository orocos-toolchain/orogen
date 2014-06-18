require 'orogen/test'

describe OroGen::Spec::Deployment do
    describe "#activity_ordered_tasks" do
        attr_reader :project, :task_model, :deployment
        before do
            loader = OroGen::Loaders::Files.new
            OroGen::Loaders::RTT.setup_loader(loader)
            @project    = OroGen::Spec::Project.new(loader)
            @project.name 'test'
            @task_model = project.task_context 'Test'
            @deployment = project.deployment 'test'
        end

        it "is well-behaved if there are no tasks in the deployment" do
            assert_equal Array.new, deployment.activity_ordered_tasks
        end
        it "places masters before the slaves" do
            slave = deployment.task('slave', task_model)
            master = deployment.task('master', task_model)
            deployment.set_master_slave_activity master, slave
            assert_equal [master, slave], deployment.activity_ordered_tasks
        end
        it "raises InternalError if there is a loop" do
            t0 = deployment.task('t0', task_model)
            t1 = deployment.task('t1', task_model)
            deployment.set_master_slave_activity t0, t1
            deployment.set_master_slave_activity t1, t0

            assert_raises(ArgumentError) { deployment.activity_ordered_tasks }
        end
    end
end
