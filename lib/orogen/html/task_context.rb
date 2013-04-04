module Orocos
    module HTML
        class TaskContext
            attr_reader :page
            attr_reader :task
            attr_reader :template

            def initialize(page)
                @template = ERB.new(File.read(File.join(File.dirname(__FILE__), "task_context_fragment.page")))
                @page = page
            end

            def render(task)
                @task = task
                page.push(nil, template.result(binding))
            end
        end
    end
end

