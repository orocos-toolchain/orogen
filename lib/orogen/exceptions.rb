module OroGen
    class NotFound < RuntimeError; end
    
    class InvalidInterfaceType < RuntimeError
        attr_reader :type

        def initialize(type)
            @type = type
        end
    end
    
    class NotExportedType < InvalidInterfaceType
        attr_reader :typekits

        def initialize(type, typekits)
            @typekits = typekits
            super(type)
        end
    end

    class ConfigError < RuntimeError
    end

    class InternalError < RuntimeError
    end
end
