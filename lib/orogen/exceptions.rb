module OroGen
    class NotFound < RuntimeError; end

    class ProjectNotFound < NotFound; end
    class TypekitNotFound < NotFound; end
    class DefinitionTypekitNotFound < TypekitNotFound; end
    class TaskModelNotFound < NotFound; end
    class TaskLibraryNotFound < NotFound; end
    class DeploymentModelNotFound < NotFound; end
    class DeployedTaskModelNotFound < NotFound; end
    class TypeNotFound < Typelib::NotFound; end
    
    class InvalidInterfaceType < RuntimeError
        attr_reader :type

        def initialize(type)
            @type = type
        end
    end
    
    # Exception thrown when a type will be used on a task context interface (to
    # create a port, property, ...), but it is exported by no typekit
    class NotExportedType < InvalidInterfaceType
        attr_reader :typekits
        def initialize(type, typekits)
            @typekits = typekits
            super(type)
        end
    end

    # Exception thrown when a type is needed but we can't find a typekit for it
    class NotTypekitType < RuntimeError
        attr_reader :type_name
        def initialize(type_name)
            @type_name = type_name
        end
    end

    class ConfigError < RuntimeError
    end

    class InternalError < RuntimeError
    end

    class AlreadyRegistered < RuntimeError
    end

    class AmbiguousName < ArgumentError
    end
end
