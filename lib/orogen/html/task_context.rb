module Orocos
    module HTML
        class TaskContext
            attr_reader :page
            attr_reader :task
            attr_reader :template

            def initialize(page)
                path = File.join(File.dirname(__FILE__), "task_context_fragment.page")
                @template = ERB.new(File.read(path))
                @template.filename = path
                @page = page
            end

            def render(task, options = Hash.new)
                @task = task
                page.push(nil, template.result(binding))
            end
        end
    end
end

