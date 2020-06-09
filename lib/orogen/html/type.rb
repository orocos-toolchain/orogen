module OroGen
    module HTML
        # Rendering object that converts a Typelib type, along with the typekit
        # information, into a HTML page.
        #
        # It needs to be given a page object which has basic services
        class Type
            attr_reader :page
            attr_reader :type

            attr_reader :intermediate_type
            attr_reader :ruby_type
            attr_reader :produced_by
            attr_reader :consumed_by
            attr_reader :displayed_by
            attr_reader :template
            attr_reader :definition_template

            def initialize(page)
                @page = page

                template_path = File.join(File.dirname(__FILE__), "type_fragment.page")
                @template = ERB.new(File.read(template_path))
                template.filename = template_path
                fragment_path = File.join(File.dirname(__FILE__), "type_definition_fragment.page")
                @definition_template = ERB.new(File.read(fragment_path))
                definition_template.filename = fragment_path

                @produced_by = []
                @consumed_by = []
                @displayed_by = []
            end

            def has_convertions?(type, recursive = true)
                if type <= Typelib::NumericType
                    return false
                elsif type.convertion_to_ruby
                    return true
                end
                return if !recursive

                if type < Typelib::CompoundType
                    type.enum_for(:each_field).any? do |field_name, field_type|
                        has_convertions?(field_type, false)
                    end
                elsif type < Typelib::EnumType
                    false
                elsif type.respond_to?(:deference)
                    return has_convertions?(type.deference, false)
                else
                    false
                end
            end

            def render_convertion_spec(base_type, convertion)
                if spec = convertion[0]
                    if spec == Array
                        # The base type is most likely an array or a container.
                        # Display the element type as well ...
                        if base_type.respond_to?(:deference)
                            if subconv = base_type.deference.convertion_to_ruby
                                return "Array(#{render_convertion_spec(base_type.deference, subconv)})"
                            else
                                return "Array(#{page.link_to(base_type.deference)})"
                            end
                        end
                    end
                    convertion[0].name

                else
                    "converted to an unspecified type"
                end
            end

            def render_type_convertion(type)
                result = []
                if convertion = type.convertion_to_ruby
                    result << render_convertion_spec(type, convertion)
                elsif type < Typelib::CompoundType
                    result << "<ul class=\"body-header-list\">"
                    type.each_field do |field_name, field_type|
                        if convertion = field_type.convertion_to_ruby
                            result << page.render_item(field_name, render_convertion_spec(field_type, convertion))
                        else
                            result << page.render_item(field_name, page.link_to(field_type))
                        end
                    end
                    result << "</ul>"
                elsif type < Typelib::ArrayType
                    result << "<ul class=\"body-header-list\">"
                    deference =
                        if convertion = type.deference.convertion_to_ruby
                            render_convertion_spec(type.deference, convertion)
                        else
                            render_convertion_spec(type.deference, [Array])
                        end
                    result << "<li>#{deference}[#{type.length}]</li>"
                    result << "</ul>"
                elsif type < Typelib::ContainerType
                    result << "<ul class=\"body-header-list\">"
                    deference =
                        if convertion = type.deference.convertion_to_ruby
                            render_convertion_spec(type.deference, convertion)
                        else
                            page.link_to(type.deference)
                        end
                    result << "<li>Array(#{deference})</li>"
                    result << "</ul>"
                else
                    raise NotImplementedError
                end
                result.join("\n")
            end

            def render_type_definition_fragment(type)
                definition_template.result(binding)
            end

            def render_type_link(type)
                if type < Typelib::ArrayType
                    "#{render_type_link(type.deference)}[#{type.length}]"
                elsif type < Typelib::ContainerType
                    "#{type.container_kind}&lt;#{render_type_link(type.deference)}&gt;"
                elsif type < Typelib::NumericType
                    type.name
                else
                    page.link_to(type)
                end
            end

            def render(type, options = Hash.new)
                _, push_options = Kernel.filter_options options, :external_objects => nil
                @type = type
                base = self.type
                typekit = begin Orocos.load_typekit_for(base, false)
                          rescue Orocos::TypekitTypeNotFound
                          end

                @intermediate_type, @ruby_type = nil
                if base.contains_opaques?
                    @intermediate_type = typekit.intermediate_type_for(type)
                    if has_convertions?(intermediate_type)
                        @ruby_type = intermediate_type
                    end
                elsif has_convertions?(base)
                    @ruby_type = base
                end

                page.push(nil, template.result(binding), push_options)
            end
        end
    end
end

