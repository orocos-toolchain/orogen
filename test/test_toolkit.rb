require 'orogen/test'

class TC_GenerationToolkit < Test::Unit::TestCase
    include Orocos::Generation::Test

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

        # Load a file with errors
        assert_raises(ArgumentError) do
            component.toolkit do
                load File.join(TEST_DATA_DIR, 'exists')
            end
        end

        # Load a file that does not exist
        assert_raises(ArgumentError) do
            component.toolkit do
                load 'does_not_exist.h'
            end
        end
    end

    def check_output_file(basedir, name)
        output   = File.read(File.join(prefix_directory, name))
        expected = File.read(File.join(TEST_DATA_DIR, basedir, name))
        assert_equal(expected, output)
    end

    def test_opaque(with_corba = true)
        build_test_component('modules/toolkit_opaque', with_corba, "bin/test") do |cmake|
            cmake << "\nADD_DEFINITIONS(-DWITH_CORBA)" if with_corba
            cmake << "\nADD_EXECUTABLE(test test.cpp)"
            cmake << "\nTARGET_LINK_LIBRARIES(test opaque-toolkit-${OROCOS_TARGET})"
            cmake << "\nTARGET_LINK_LIBRARIES(test ${OROCOS_COMPONENT_LIBRARIES})"
            cmake << "\nINSTALL(TARGETS test RUNTIME DESTINATION bin)"
            cmake << "\n"
	end

        check_output_file('modules/toolkit_opaque', 'opaque.xml')
        check_output_file('modules/toolkit_opaque', 'opaque.cpf')
        check_output_file('modules/toolkit_opaque', 'composed_opaque.xml')
        check_output_file('modules/toolkit_opaque', 'composed_opaque.cpf')
        check_output_file('modules/toolkit_opaque', 'shared_ptr__opaque_type.xml')
        check_output_file('modules/toolkit_opaque', 'shared_ptr__opaque_type.cpf')
        check_output_file('modules/toolkit_opaque', 'shared_ptr__shared_ptr.xml')
        check_output_file('modules/toolkit_opaque', 'shared_ptr__shared_ptr.cpf')
        check_output_file('modules/toolkit_opaque', 'readonlypointer.xml')
        check_output_file('modules/toolkit_opaque', 'readonlypointer.cpf')
    end
    def test_opaque_without_corba; test_opaque(false) end

    def test_opaque_autodef(with_corba = true)
        build_test_component('modules/toolkit_autodef', with_corba)
    end

    def test_opaque_validation
        # First, check that the actual opaque module generates properly
        create_wc("modules/toolkit_opaque_validation_ok")
        FileUtils.rm_rf working_directory
        FileUtils.cp_r File.join(TEST_DATA_DIR, "modules/toolkit_opaque"), working_directory

        component = Component.new
        in_wc do
            component.load 'opaque.orogen'
            assert_nothing_raised { component.generate }
        end

        # Second, check that it fails if an invalid file is loaded
        create_wc("modules/toolkit_opaque_validation_fail")
        FileUtils.rm_rf working_directory
        FileUtils.cp_r File.join(TEST_DATA_DIR, "modules/toolkit_opaque"), working_directory

        component = Component.new
        in_wc do
            component.load 'opaque.orogen'
            component.toolkit.load File.join(TEST_DATA_DIR, 'opaque_invalid.h')
            assert_raises(NotImplementedError) { component.generate }
        end
    end

    def test_simple(with_corba = true)
        build_test_component('modules/toolkit_simple', with_corba, "bin/test") do |cmake|
            cmake << "ADD_DEFINITIONS(-DWITH_CORBA)\n" if with_corba
            cmake << <<-EOT
link_directories(${OrocosCORBA_LIBDIR} ${OrocosRTT_LIBDIR})
ADD_EXECUTABLE(test test.cpp)
TARGET_LINK_LIBRARIES(test simple-toolkit-${OROCOS_TARGET}
    simple-transport-corba-${OROCOS_TARGET})
TARGET_LINK_LIBRARIES(test ${OrocosRTT_LIBRARIES} ${OrocosCORBA_LIBRARIES})
INSTALL(TARGETS test RUNTIME DESTINATION bin)
            EOT
        end

        check_output_file('modules/toolkit_simple', 'basic.cpf')
        check_output_file('modules/toolkit_simple', 'basic.xml')
        check_output_file('modules/toolkit_simple', 'simple_vector.cpf')
        check_output_file('modules/toolkit_simple', 'simple_vector.xml')
        check_output_file('modules/toolkit_simple', 'complex_vector.cpf')
        check_output_file('modules/toolkit_simple', 'complex_vector.xml')
        check_output_file('modules/toolkit_simple', 'complex_array.cpf')
        check_output_file('modules/toolkit_simple', 'complex_array.xml')
    end
    def test_simple_without_corba; test_simple(false) end
end

