require 'erb'
require 'typelib'

module Typelib
    class Type
	def self.to_orocos_decomposition(result, path, indent = "    ")
	    orocos_type = registry.orocos_equivalent(self).basename
	    result << indent << "target_bag.add( new Property<#{orocos_type}>(\"#{path}\", \"\", value#{path}) );"
	end
	def self.to_orocos_composition
	end
    end
    class CompoundType
	def self.to_orocos_decomposition(result, path, indent = "    ")
	    each_field do |name, type|
		type.to_orocos_decomposition(result, "#{path}.#{name}", indent) << "\n"
	    end
	end
	def self.to_orocos_composition
	end
    end
    class ArrayType
	def self.to_orocos_decomposition(result, path, indent = "    ")
	    result << indent << "for(i = 0; i < #{length}; ++i) {\n" 
	    deference.to_orocos_decomposition(result, path + "[i]", indent + "  ") << "\n"
	    result << indent << "}"
	end
	def self.to_orocos_composition
	end
    end
    class Registry
	OROCOS_KNOWN_TYPES = ['char', 'int', 'unsigned int', 'float', 'double']
	OROCOS_KNOWN_CONVERTIONS = {
	    'short' => 'int',
	    'unsigned short' => 'unsigned int' }

	attr_reader :orocos_type_equivalence
	def build_orocos_type_equivalence
	    @orocos_type_equivalence = Hash.new

	    orocos_known_types = OROCOS_KNOWN_TYPES.map { |name| get(name) }
	    orocos_known_convertions = OROCOS_KNOWN_CONVERTIONS.map { |from, to| [get(from), get(to)] }
	    each_type do |type|
		next if type < CompoundType || type < ArrayType

		if eqv = orocos_known_types.find { |known_t| known_t == type }
		    orocos_type_equivalence[type] = eqv
		elsif eqv = orocos_known_convertions.find { |from_t, to_t| from_t == type }
		    orocos_type_equivalence[type] = eqv[1]
		end
	    end
	end

	def orocos_equivalent(user_type)
	    if !orocos_type_equivalence
		build_orocos_type_equivalence
	    end

	    if type = orocos_type_equivalence[user_type]
		type
	    else
		raise TypeError, "#{user_type.name} does not have an equivalent in the Orocos RTT toolkit"
	    end
	end

	def to_orocos_toolkit(result)
	    type_info = Generation.load_template("type_info.cpp")

	    each_type do |type|
		if type < CompoundType
		    result << type_info.result(binding)
		end
	    end
	end
    end
end

module Generation
    @templates = Hash.new
    class << self
	attr_reader :templates
    end

    def self.load_template(name)
	if template = templates[name]
	    template
	else
	    template_basedir = File.expand_path('templates', File.dirname(__FILE__))
	    template_data    = File.open(File.join(template_basedir, name))
	    templates[name] = ERB.new(template_data.read)
	end
    end

    def self.to_orocos_toolkit(header)
	registry = Typelib::Registry.new
	registry.import(header)
	puts registry.to_orocos_toolkit("")
    end
end

