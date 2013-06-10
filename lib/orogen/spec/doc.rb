module Orocos
    module Spec
        def self.load_documentation(object, defmethod)
            if doc = MetaRuby::DSLs.parse_documentation_block(/\.orogen$/, defmethod)
                object.doc(doc)
            end
        end
    end
end

