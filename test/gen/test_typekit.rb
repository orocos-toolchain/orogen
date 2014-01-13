require 'orogen/test'

describe Orocos::Generation::Typekit do

    describe "#filter_unsupported_types" do
        attr_reader :typekit
        before do
            @typekit = Orocos::Generation::Typekit.new
        end

        it "rejects std::vector<bool>" do
            reg = Typelib::Registry.import File.join(Orocos::Generation::Test::TEST_DATA_DIR, 'typekit', 'std_vector_bool.h')
            typekit.filter_unsupported_types(reg)
            assert !reg.include?("/std/vector</bool>")
            assert !reg.include?("/StructWithVectorBool")
            assert !reg.include?("/VectorWithVectorBool")
        end
    end
end
