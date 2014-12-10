require 'orogen/test'

describe Orocos::Spec::Port do
    include Orocos::Generation::Test
    describe "#to_h" do
        attr_reader :task, :p
        before do
            @task = Orocos::Spec::TaskContext.new(Orocos::Generation::Project.new)
            @p = task.output_port('out', '/double')
        end

        it "marshals the port name" do
            assert_equal 'out', p.to_h[:name]
        end
        it "marshals the port direction" do
            out_p = task.output_port('out_p', '/double')
            assert_equal 'output', out_p.to_h[:direction]
            in_p  = task.input_port('in_p', '/double')
            assert_equal 'input', in_p.to_h[:direction]
        end
        it "marshals the port type" do
            assert_equal p.type.to_h, p.to_h[:type]
        end
        it "marshals empty port documentation as an empty string" do
            p = task.output_port 'with_no_documentation', '/double'
            assert_equal "", p.to_h[:doc]
        end
        it "marshals the port documentation" do
            p = task.output_port('with_no_documentation', '/double').
                doc('port with documentation')
            assert_equal "port with documentation", p.to_h[:doc]
        end
    end
end

