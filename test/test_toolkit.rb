require 'orogen/test'

class TC_GenerationToolkit < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

    def test_orocos_type_equivalence
	registry = Typelib::Registry.new

	assert_equal(registry.get('int'), registry.orocos_equivalent(registry.get('int32_t')))
	assert_equal(registry.get('unsigned int'), registry.orocos_equivalent(registry.get('uint32_t')))
	assert_equal(registry.get('int'), registry.orocos_equivalent(registry.get('int16_t')))
	assert_equal(registry.get('unsigned int'), registry.orocos_equivalent(registry.get('uint16_t')))
	assert_equal(registry.get('int'), registry.orocos_equivalent(registry.get('short')))
	assert_equal(registry.get('unsigned int'), registry.orocos_equivalent(registry.get('unsigned short')))

	assert_raises(TypeError) { registry.orocos_equivalent(registry.get('int64_t')) }
    end

    def test_toolkit_load
	component = Component.new
        component.name 'test_toolkit_load'

        assert_raises(RuntimeError) do
            component.toolkit do
                load File.join(TEST_DATA_DIR, 'exists')
            end
        end

        assert_raises(ArgumentError) do
            component.toolkit do
                load 'does_not_exist.h'
            end
        end
    end

    def check_output_file(name)
        output   = File.read(name)
        expected = File.read(File.join(TEST_DATA_DIR, name))
        assert_equal(expected, output)
    end

    def test_opaque(with_corba = true)
        copy_in_wc File.join(TEST_DATA_DIR, 'test_toolkit_opaque.orogen')
        copy_in_wc File.join(TEST_DATA_DIR, 'test_toolkit_opaque.cpp')
        copy_in_wc File.join(TEST_DATA_DIR, 'opaque.h')
        copy_in_wc File.join(TEST_DATA_DIR, 'opaque_intermediates.h')

        component = Component.new
        in_wc do
            component.load 'test_toolkit_opaque.orogen'
            if with_corba
                component.enable_corba
            else
                component.disable_corba
            end

            component.generate
        end

        copy_in_wc File.join(TEST_DATA_DIR, 'TestOpaqueToolkitUser.cpp'), 'toolkit'
        compile_and_test(component, 'bin/test_toolkit') do |cmake|
            cmake << "\nADD_DEFINITIONS(-DWITH_CORBA)" if with_corba
            cmake << "\nADD_EXECUTABLE(test_toolkit test_toolkit_opaque.cpp)"
            cmake << "\nTARGET_LINK_LIBRARIES(test_toolkit TestOpaque-toolkit-${OROCOS_TARGET})"
            cmake << "\nTARGET_LINK_LIBRARIES(test_toolkit ${OROCOS_COMPONENT_LIBRARIES})"
            cmake << "\nINSTALL(TARGETS test_toolkit RUNTIME DESTINATION bin)"
            cmake << "\n"
	end

	in_prefix do
            check_output_file('test_toolkit_opaque.xml')
            check_output_file('test_toolkit_opaque.cpf')
            check_output_file('test_toolkit_composed_opaque.xml')
            check_output_file('test_toolkit_composed_opaque.cpf')
	end
    end
    def test_opaque_without_corba; test_opaque(false) end

    def test_generation(with_corba = true)
	component = Component.new
        component.load File.join(TEST_DATA_DIR, 'test_toolkit_generation.orogen')
        if with_corba
            component.enable_corba
        else
            component.disable_corba
        end

	copy_in_wc File.join(TEST_DATA_DIR, 'test_toolkit.cpp')
	in_wc do
	    component.generate
	    if with_corba
		assert(File.file?( File.join(Generation::AUTOMATIC_AREA_NAME, 'toolkit', 'TestToolkitCorba.hpp')))
		assert(File.file?( File.join(Generation::AUTOMATIC_AREA_NAME, 'toolkit', 'TestToolkit.idl')))
	    else
		assert(!File.file?( File.join(Generation::AUTOMATIC_AREA_NAME, 'toolkit', 'TestToolkitCorba.hpp')))
		assert(!File.file?( File.join(Generation::AUTOMATIC_AREA_NAME, 'toolkit', 'TestToolkit.idl')))
	    end
	    assert(File.file?( File.join(Generation::AUTOMATIC_AREA_NAME, 'toolkit', 'TestToolkit.hpp')))
	    assert(File.file?( File.join(Generation::AUTOMATIC_AREA_NAME, 'toolkit', 'TestToolkit.cpp')))
	end

        compile_and_test(component, 'bin/test_toolkit') do |cmake|
            cmake << "\nADD_DEFINITIONS(-DWITH_CORBA)" if with_corba
            cmake << "\nADD_EXECUTABLE(test_toolkit test_toolkit.cpp)"
            cmake << "\nTARGET_LINK_LIBRARIES(test_toolkit Test-toolkit-${OROCOS_TARGET})"
            cmake << "\nTARGET_LINK_LIBRARIES(test_toolkit ${OROCOS_COMPONENT_LIBRARIES})"
            cmake << "\nINSTALL(TARGETS test_toolkit RUNTIME DESTINATION bin)"
            cmake << "\n"
	end

	in_prefix do
            check_output_file('test_toolkit.xml')
	end
    end
    def test_generation_without_corba; test_generation(false) end
end

