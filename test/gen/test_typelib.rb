require 'orogen/gen/test'

describe Typelib do
    attr_reader :registry

    before do
        @registry = Typelib::CXXRegistry.new
    end

    describe Typelib::NumericType do
        describe "#cxx_name" do
            it "returns the equivalent boost type" do
                assert_equal 'boost::int32_t', registry.get('/int32_t').cxx_name
            end
        end
    end
end
