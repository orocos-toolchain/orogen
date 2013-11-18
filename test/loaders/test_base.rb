require 'orogen/test'

describe OroGen::Loaders::Base do
    include FlexMock::ArgumentTypes
    include FlexMock::MockContainer

    attr_reader :loader

    before do
        @loader = flexmock(OroGen::Loaders::Base.new)
        loader.loaded_task_models.merge! OroGen::Loaders::RTT.standard_tasks
    end

    describe "#project_model_from_name" do
        before do
            loader.should_receive(:has_typekit?).
                and_return(false).by_default
            loader.should_receive(:has_project?).
                and_return(false).by_default
        end
        it "should build the model from text" do
            orogen_model =<<-EOMODEL
            name "test"
            task_context "Task" do
            end
            EOMODEL
            flexmock(loader).should_receive(:project_model_text_from_name).
                with('test').and_return([orogen_model, nil])

            model = loader.project_model_from_name('test')
            assert_equal 'test', model.name
            assert model.self_tasks['test::Task']
        end
    end
end


