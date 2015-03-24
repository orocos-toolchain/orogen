require 'set'

module Orocos
    module Generation
        class CppProxyGeneration
            attr_reader :project
            attr_reader :proxy_gen
            attr_reader :task
            def initialize(cmp, t)
                @project = cmp
                @task = t
                @proxy_gen = self
            end
            
            def dependencies(includeSelf = true)
                list = project.used_typekits.dup
                if(includeSelf)
                    list << project.typekit if project.typekit
                end
                result = []
                list.each do |tk|
                    next if tk.name == "rtt"
                    next if tk.name == "logger"
                    result << "#{tk.name}-typekit-gnulinux"
                    project.enabled_transports.each do |transport_name|
                        result << "#{tk.name}-transport-#{transport_name}-gnulinux"
                    end
                end
                result
            end

            # Generates the code associated with cpp proxies
            def generate
                proxy_hpp = Generation.render_template "proxies", "Task.hpp", binding
                proxy_cpp = Generation.render_template "proxies", "Task.cpp", binding
                file_proxy_hpp = Generation.save_automatic "proxies", "#{task.basename}.hpp", proxy_hpp
                file_proxy_cpp = Generation.save_automatic "proxies", "#{task.basename}.cpp", proxy_cpp

                cmake = Generation.render_template 'proxies', 'CMakeLists.txt', binding
                Generation.save_automatic('proxies', "CMakeLists.txt", cmake)
                
                pc = Generation.render_template "proxies", "proxies.pc", binding
                Generation.save_automatic "proxies", "#{project.name}-proxies.pc.in", pc
            end
        end
    end
end

