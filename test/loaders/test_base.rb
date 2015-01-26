require 'orogen/test'

describe OroGen::Loaders::Base do
    attr_reader :loader

    before do
        @loader = flexmock(OroGen::Loaders::Base.new)
        loader.should_receive(:has_typekit?).
            and_return(false).by_default
        loader.should_receive(:has_project?).
            and_return(false).by_default
    end

    describe "#project_model_from_name" do
        before do
            OroGen::Loaders::RTT.setup_loader(loader)
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

    describe "#typekit_model_from_name" do
        attr_reader :typekit
        before do
            tlb =<<-EOF
<?xml version="1.0"?>
<typelib>
  <container name="/std/string" of="/int8_t" size="0" kind="/std/string" />
  <alias name="/string" source="/std/string"/>
</typelib>
            EOF
            typelist=<<-EOF
            /string 1
            EOF
            loader.should_receive(:typekit_model_text_from_name).
                with('test').and_return([tlb, typelist])

            @typekit = loader.typekit_model_from_name('test')
        end

        it "should register the type-to-typekit mapping" do
            assert_equal [typekit].to_set, loader.typekits_by_type_name['/string']
        end
        it "should register all interface types" do
            assert_equal ['/string'].to_set, loader.interface_typelist
        end
    end
end


