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

    describe "#deployed_task_model_from_name" do
        it "raises AmbiguousName if there are more than one deployment with a task matching the name" do
            loader.should_receive(:find_deployments_from_deployed_task_name).
                with(task_name = flexmock).
                once.
                and_return([flexmock(name: 'a'), flexmock(name: 'b')])
            assert_raises(OroGen::AmbiguousName) { loader.deployed_task_model_from_name(task_name) }
        end
    end

    describe "#imported_typekits_for" do
        attr_reader :registry, :typekit, :definition_typekit
        before do
            @registry = Typelib::CXXRegistry.new
            @typekit = OroGen::Spec::Typekit.new(loader, "typekit", registry, [])
            @definition_typekit = OroGen::Spec::Typekit.new(loader, "definition_typekit", registry, ['/int32_t'])
            loader.register_typekit_model(typekit)
        end

        it "raises DefinitionTypekitNotFound if no loaded typekits define the type" do
            assert_raises(OroGen::DefinitionTypekitNotFound) do
                loader.imported_typekits_for("/Test")
            end
        end

        describe "definition_typekits: false" do
            it "returns the typekits that have the type in their registry" do
                loader.register_typekit_model(definition_typekit)
                assert_equal [typekit, definition_typekit].to_set,
                    loader.imported_typekits_for('/int32_t', definition_typekits: false)
            end
        end
        describe "definition_typekits: true" do
            it "returns the typekits that directly define the type" do
                loader.register_typekit_model(definition_typekit)
                assert_equal [definition_typekit].to_set,
                    loader.imported_typekits_for('/int32_t', definition_typekits: true)
            end
            it "raises DefinitionTypekitNotFound if some typekits define the type, but none of them directly define it" do
                assert_raises(OroGen::DefinitionTypekitNotFound) do
                    loader.imported_typekits_for("/int32_t", definition_typekits: true)
                end
            end
        end
    end
end


