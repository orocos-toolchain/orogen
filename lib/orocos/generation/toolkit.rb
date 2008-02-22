require 'typelib'
require 'orocos/generation/base'

module Typelib
    class Type
	def self.to_orocos_decomposition(result, path, indent = "    ")
	    orocos_type = registry.orocos_equivalent(self).basename
	    result << indent << "target_bag.add( new Property<#{orocos_type}>(\"#{path}\", \"\", value#{path}) );"
	end

	def self.code_to_corba(result, path = "", indent = "    ")
	    result << indent << "result#{path} = value#{path};"
	end
	def self.code_from_corba(result, path = "", indent = "    ")
	    code_to_corba(result, path, indent)
	end

	def self.to_orocos_composition
	end
    end
    class CompoundType
	def self.convertion_code_helper(method, result, path, indent)
	    each_field do |name, type|
		type.send(method, result, "#{path}.#{name}", indent) << "\n"
	    end
	end
	def self.to_orocos_decomposition(result, path, indent = "    ")
	    convertion_code_helper(:to_orocos_decomposition, result, path, indent)
	end
	def self.code_to_corba(result, path = "", indent = "    ")
	    convertion_code_helper(:code_to_corba, result, path, indent)
	end

	def self.to_orocos_composition
	end
    end
    class ArrayType
	def self.convertion_code_helper(method, result, path, indent)
	    result << indent << "for (i = 0; i < #{length}; ++i) {\n" 
	    deference.send(method, result, path + "[i]", indent + "    ") << "\n"
	    result << indent << "}"
	end
	def self.to_orocos_decomposition(result, path, indent = "    ")
	    convertion_code_helper(:to_orocos_decomposition, result, path, indent)
	end
	def self.code_to_corba(result, path = "", indent = "    ")
	    convertion_code_helper(:code_to_corba, result, path, indent)
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

	# Generates the code for the toolkit plugin which handles the types
	# found in +registry+, and returns it as header, source. The corresponding
	# header file is supposed to be named ${toolkit_name}Toolkit.hpp
	def to_orocos_toolkit(toolkit_name)
	    generated_types = []
	    each_type do |type|
		if type < CompoundType
		    generated_types << type
		end
	    end

	    corba  = Orocos::Generation.render_template 'toolkit/corba.hpp', binding
	    header = Orocos::Generation.render_template "toolkit/header.hpp", binding
	    source = Orocos::Generation.render_template "toolkit/toolkit.cpp", binding

	    return corba, header, source
	end
    end
end

module Orocos
    module Generation
	class Toolkit
	    attr_reader :name, :imports, :loads
	    attr_reader :registry
	    def self.validate_name(name)
		if !name || name.empty?
		    raise "empty toolkit name"
		elsif name !~ /^\w+$/
		    raise "toolkit names can only contain alphanumeric characters and _"
		end

		# TODO: check that this toolkit name is not already used ?
	    end

	    def initialize(name)
		Toolkit.validate_name name
		@name = name

		@imports, @loads = [], []
		@registry = Typelib::Registry.new
	    end

	    def load(file)
		file = File.expand_path(file)
		loads << file
		registry.import(file)
	    end

	    def import(other_toolkit)
		raise NotImplementedError
	    end

	    def to_code
		type_header = Orocos::Generation.render_template('toolkit/types.hpp', binding)
		corba, hpp, cpp = registry.to_orocos_toolkit(name)

		return type_header, corba, hpp, cpp
	    end
	end

	def self.toolkit(name, &block)
	    toolkit = Toolkit.new(name)
	    toolkit.instance_eval(&block)

	    types, corba, hpp, cpp = toolkit.to_code
	    save_automatic("toolkit", "#{name}ToolkitTypes.hpp", types)
	    save_automatic("toolkit", "#{name}ToolkitCorba.hpp", corba)
	    save_automatic("toolkit", "#{name}Toolkit.hpp", hpp)
	    save_automatic("toolkit", "#{name}Toolkit.cpp", cpp)
	end
    end
end

