require 'orogen/test'

describe OroGen::Spec::ConfigurationObject do
    describe "#to_h" do
        attr_reader :task, :p
        before do
            @task = OroGen::Spec::TaskContext.new(create_dummy_project)
            @p = task.property('p', '/double')
        end

        it "marshals the name" do
            assert_equal 'p', p.to_h[:name]
        end
        it "marshals the type" do
            assert_equal p.type.to_h, p.to_h[:type]
        end
        it "marshals whether the object is dynamic or not" do
            assert_equal false, p.to_h[:dynamic]
            flexmock(p).should_receive(:dynamic?).and_return(true)
            assert_equal true, p.to_h[:dynamic]
        end
        it "marshals empty documentation as an empty string" do
            p = task.property 'with_no_documentation', '/double'
            assert_equal "", p.to_h[:doc]
        end
        it "marshals the documentation" do
            p = task.property('with_documentation', '/double').
                doc('with documentation')
            assert_equal "with documentation", p.to_h[:doc]
        end
        it "does not add a default field if the configuration object has no default value" do
            assert !p.to_h.has_key?(:default)
        end
        it "marshals the default value if there is one" do
            flexmock(p).should_receive(:default_value).and_return(v = flexmock)
            assert_equal v, p.to_h[:default]
        end
    end
end
