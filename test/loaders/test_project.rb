require 'orogen/test'

describe OroGen::Loaders::Project do
    attr_reader :loader, :target
    before do
        @loader = OroGen::Loaders::Base.new
        OroGen::Loaders::RTT.setup_loader(loader)
        @target = OroGen::Spec::Project.new(loader)
        target.define_default_deployments = false
    end

    def load(file)
        OroGen::Loaders::Project.new(target).__load__(file)
    end

    it "loads documentation for each task context object" do
        load(File.join(path_to_data, 'modules', 'load_documentation.orogen'))

        task = target.task_model_from_name('Task')
        assert_equal 'TASKCONTEXT', task.doc
        assert_equal 'PROPERTY', task.find_property('p').doc
        assert_equal 'INPUTPORT', task.find_input_port('i').doc
        assert_equal 'OUTPUTPORT', task.find_output_port('o').doc
        assert_equal 'OPERATION', task.find_operation('op').doc
    end

    it "does not load after an empty line" do
        load(File.join(path_to_data, 'modules', 'load_documentation.orogen'))
        task = target.task_model_from_name('Task')
        assert_equal 'PROPERTY', task.find_property('test_empty_line_after_proper_comment').doc
        assert_equal nil, task.find_property('test_empty_line').doc
    end
end
