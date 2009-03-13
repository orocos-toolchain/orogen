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

        assert_raises(RuntimeError) do
            component.toolkit do
                load 'does_not_exist.h'
            end
        end
    end

    def check_output_file(basedir, name)
        output   = File.read(File.join(TEST_DIR, "wc", "prefix", name))
        expected = File.read(File.join(TEST_DATA_DIR, basedir, name))
        assert_equal(expected, output)
    end

    def test_opaque(with_corba = true)
        build_test_component(File.join(TEST_DATA_DIR, 'modules/opaque'), with_corba, "bin/test") do |cmake|
            cmake << "\nADD_DEFINITIONS(-DWITH_CORBA)" if with_corba
            cmake << "\nADD_EXECUTABLE(test test.cpp)"
            cmake << "\nTARGET_LINK_LIBRARIES(test opaque-toolkit-${OROCOS_TARGET})"
            cmake << "\nTARGET_LINK_LIBRARIES(test ${OROCOS_COMPONENT_LIBRARIES})"
            cmake << "\nINSTALL(TARGETS test RUNTIME DESTINATION bin)"
            cmake << "\n"
	end

        check_output_file('modules/opaque', 'opaque.xml')
        check_output_file('modules/opaque', 'opaque.cpf')
        check_output_file('modules/opaque', 'composed_opaque.xml')
        check_output_file('modules/opaque', 'composed_opaque.cpf')
    end
    def test_opaque_without_corba; test_opaque(false) end

    def test_opaque_validation
        # Copy in place of wc
        wc_dirname = File.join(TEST_DIR, "wc")
        FileUtils.rm_rf wc_dirname
        FileUtils.cp_r File.join(TEST_DATA_DIR, "modules/opaque"), wc_dirname

        component = Component.new
        in_wc do
            component.load 'opaque.orogen'
            assert_nothing_raised { component.generate }
        end
        clear_wc

        wc_dirname = File.join(TEST_DIR, "wc")
        FileUtils.rm_rf wc_dirname
        FileUtils.cp_r File.join(TEST_DATA_DIR, "modules/opaque"), wc_dirname

        component = Component.new
        in_wc do
            component.load 'opaque.orogen'
            component.toolkit.load File.join(TEST_DATA_DIR, 'opaque_invalid.h')
            assert_raises(NotImplementedError) { component.generate }
        end
        clear_wc
    end

    def test_simple(with_corba = true)
        build_test_component(File.join(TEST_DATA_DIR, 'modules/simple'), with_corba, "bin/test") do |cmake|
             cmake << "\nADD_DEFINITIONS(-DWITH_CORBA)" if with_corba
             cmake << "\nADD_EXECUTABLE(test test.cpp)"
             cmake << "\nTARGET_LINK_LIBRARIES(test simple-toolkit-${OROCOS_TARGET})"
             cmake << "\nTARGET_LINK_LIBRARIES(test ${OROCOS_COMPONENT_LIBRARIES})"
             cmake << "\nINSTALL(TARGETS test RUNTIME DESTINATION bin)"
             cmake << "\n"
        end

        check_output_file('modules/simple', 'simple.cpf')
        check_output_file('modules/simple', 'simple.xml')
    end
    def test_simple_without_corba; test_simple(false) end
end

