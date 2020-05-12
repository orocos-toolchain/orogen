# frozen_string_literal: true

require "orogen/test"

describe OroGen::Loaders::Aggregate do
    before do
        @loader = OroGen::Loaders::Aggregate.new
    end

    describe "#has_project?" do
        it "returns false by default" do
            refute @loader.has_project?("does_not_exist")
        end
        it "returns true if the project exists on one of its children" do
            @loader.add(child = OroGen::Loaders::Base.new)
            flexmock(child).should_receive(:has_project?).with("test").and_return(true)
            assert @loader.has_project?("test")
        end
        it "returns true if the project has been directly registered" do
            @loader.project_model_from_text(<<-END)
            name 'directly_registered'
            END
            assert @loader.has_project?("directly_registered")
        end
    end

    describe "#has_typekit?" do
        it "returns false by default" do
            refute @loader.has_typekit?("does_not_exist")
        end
        it "returns true if the typekit exists on one of its children" do
            @loader.add(child = OroGen::Loaders::Base.new)
            flexmock(child).should_receive(:has_typekit?).with("test").and_return(true)
            assert @loader.has_typekit?("test")
        end
        it "returns true if the typekit has been directly registered" do
            typekit = OroGen::Spec::Typekit.new(nil, "directly_registered")
            @loader.register_typekit_model(typekit)
            assert @loader.has_typekit?("directly_registered")
        end
    end
end
