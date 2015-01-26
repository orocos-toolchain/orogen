module OroGen
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
                options, push_options = Kernel.filter_options options, :doc => true, :external_objects => nil
                @task = task
                if options[:doc] && task.doc
                    page.push nil, page.main_doc(task.doc)
                end
                page.push(nil, template.result(binding), push_options)
            end
        end
    end
end

