require 'orogen/test'

describe OroGen::Spec::Port do
    attr_reader :project, :loader, :task, :port
    before do
        @loader = OroGen::Loaders::Files.new
        OroGen::Loaders::RTT.setup_loader(loader)
        @project = OroGen::Spec::Project.new(loader)
        project.name 'base'
        @task = project.task_context 'Task'
        @port = OroGen::Spec::Port.new(task, 'test', '/double')
    end

    describe "#to_h" do
        it "marshals the port name" do
            assert_equal 'test', port.to_h[:name]
        end
        it "marshals the port direction" do
            out_p = task.output_port('out_p', '/double')
            assert_equal 'output', out_p.to_h[:direction]
            in_p  = task.input_port('in_p', '/double')
            assert_equal 'input', in_p.to_h[:direction]
        end
        it "marshals the port type" do
            assert_equal port.type.to_h, port.to_h[:type]
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

    describe "static or dynamic connection" do
        it "is dynamic by default" do
            assert !port.static?
        end
        it "becomes static if #static is called" do
            port.static
            assert port.static?
        end
        it "becomes dynamic again if #dynamic is called" do
            port.static
            port.dynamic
            assert !port.static?
        end
    end
end

