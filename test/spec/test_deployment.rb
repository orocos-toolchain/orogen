require 'orogen/test'

describe OroGen::Spec::Deployment do
    attr_reader :project, :task_model, :deployment
    before do
        loader = OroGen::Loaders::Files.new
        OroGen::Loaders::RTT.setup_loader(loader)
        @project    = OroGen::Spec::Project.new(loader)
        @project.name 'test'
        @task_model = project.task_context 'Test'
        @deployment = project.deployment 'test'
    end

    describe "#activity_ordered_tasks" do
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

    describe "#task" do
        it "sets the task name to the given name" do
            task = deployment.task('task', 'Test')
            assert_equal 'task', task.name
        end
        it "sets the task model to the given model" do
            task = deployment.task('task', 'Test')
            assert_equal task_model, task.task_model
        end
        it "accepts a task model by object" do
            task = deployment.task('task', task_model)
            assert_equal task_model, task.task_model
        end
        it "raises ArgumentError if the model name cannot be resolved" do
            assert_raises(OroGen::TaskModelNotFound) { deployment.task "name", "Bla" }
        end
        it "raises ArgumentError if a task with the given name already exists" do
            deployment.task('task', task_model)
            assert_raises(ArgumentError) { deployment.task "task", task_model }
        end
        it "sets the deployed task's activity to the default" do
            task_model.default_activity :periodic, 0.1
            task = deployment.task "test", task_model
            assert_equal("RTT::Activity", task.activity_type.class_name)
            assert_equal(0.1, task.period)
        end
    end
end

describe OroGen::Spec::TaskDeployment do
    attr_reader :project, :task_model, :deployment
    before do
        loader = OroGen::Loaders::Files.new
        OroGen::Loaders::RTT.setup_loader(loader)
        @project    = OroGen::Spec::Project.new(loader)
        @project.name 'test'
        @task_model = project.task_context 'Test'
        @deployment = project.deployment 'test'
    end

    it "allows to change a default activity" do
        task_model.default_activity :periodic, 0.1
        task = deployment.task "my_name", task_model
        task.triggered
        assert_equal("RTT::Activity", task.activity_type.class_name)
        assert_equal(0, task.period)
    end

    it "raises ArgumentError if trying to change a required activity" do
        task_model.required_activity :periodic, 0.1
        task       = deployment.task "my_name", task_model
        assert_raises(ArgumentError) { task.triggered }
    end

    it "raises ArgumentError if trying to change an already explicitly set activity" do
        task = deployment.task "my_name", task_model
        task.triggered
        assert_raises(ArgumentError) { task.periodic(0.1) }
    end
end

describe OroGen::Spec::ConnPolicy do
    describe ".from_hash" do
        def conn_policy; OroGen::Spec::ConnPolicy end

        it "should assign hash values to variables" do
            policy = conn_policy.from_hash :type => :buffer, :lock_policy => :locked, :size => 10
            assert_equal(:buffer, policy.type)
            assert_equal(:locked, policy.lock_policy)
            assert_equal(false, policy.pull)
            assert_equal(10, policy.size)
        end
        it "should set sane defaults if given an empty hash" do
            policy = conn_policy.from_hash(Hash.new)
            assert_equal(:data, policy.type)
            assert_equal(:lock_free, policy.lock_policy)
            assert_equal(false, policy.pull)
            assert_equal(0, policy.size)
        end
        it "should raise ArgumentError if the connection type is unknown" do
            assert_raises(ArgumentError) { conn_policy.from_hash :type => :unknown }
        end
        it "should raise ArgumentError if the lock_policy is unknown" do
            assert_raises(ArgumentError) { conn_policy.from_hash :lock_policy => :unknown }
        end
        it "should raise ArgumentError if size is given for a data connection" do
            assert_raises(ArgumentError) { conn_policy.from_hash :type => :data, :size => 10 }
            assert_raises(ArgumentError) { conn_policy.from_hash :type => :data, :size => 0 }
        end
        it "should raise ArgumentError if size is not given for a buffer connection" do
            assert_raises(ArgumentError) { conn_policy.from_hash :type => :buffer }
        end
        it "should raise ArgumentError if a non-positive size is given for a buffer connection" do
            assert_raises(ArgumentError) { conn_policy.from_hash :type => :buffer, :size => 0 }
            assert_raises(ArgumentError) { conn_policy.from_hash :type => :buffer, :size => -1 }
        end
    end
end

