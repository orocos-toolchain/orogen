require 'orogen/test'

describe Orocos::Generation::Typekit do

    describe "#filter_unsupported_types" do
        attr_reader :typekit
        before do
            @typekit = Orocos::Generation::Typekit.new
        end

        it "rejects multi-dimensional arrays" do
            reg = Typelib::Registry.import File.join(Orocos::Generation::Test::TEST_DATA_DIR, 'typekit', 'multi_dimensional_array.h')
            typekit.filter_unsupported_types(reg)
            assert !reg.include?("/double[2][4]")
        end
        it "rejects std::vector<bool>" do
            reg = Typelib::Registry.import File.join(Orocos::Generation::Test::TEST_DATA_DIR, 'typekit', 'std_vector_bool.h')
            typekit.filter_unsupported_types(reg)
            assert !reg.include?("/std/vector</bool>")
        end
        it "rejects pointers" do
            reg = Typelib::Registry.import File.join(Orocos::Generation::Test::TEST_DATA_DIR, 'typekit', 'pointer.h')
            typekit.filter_unsupported_types(reg)
            assert !reg.include?("/double*")
        end
        it "rejects compounds whose field name does not start with an alphanumeric character" do
            reg = Typelib::Registry.import File.join(Orocos::Generation::Test::TEST_DATA_DIR, 'typekit', 'compound_with_field_not_starting_with_alphanumeric_character.h')
            typekit.filter_unsupported_types(reg)
            assert !reg.include?("/Test")
        end
        it "rejects the types that depend on rejected types" do
            reg = Typelib::Registry.import File.join(Orocos::Generation::Test::TEST_DATA_DIR, 'typekit', 'rejected_dependencies.h')
            typekit.filter_unsupported_types(reg)
            assert !reg.include?("/CompoundTest")
            assert !reg.include?("/VectorTest")
            assert !reg.include?("/ArrayTest")
        end
    end
end
