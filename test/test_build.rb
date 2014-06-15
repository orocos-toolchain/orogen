$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orogen/test'

class TC_GenerationBuild < Minitest::Test
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

    def test_check_uptodate
        build_test_component "modules/simple", []

        in_wc do
            # Touch the orogen file
            FileUtils.touch "simple_component.orogen"

            # First check that the user is forbidden to go on with building
            Dir.chdir("build") do
                assert !system("make")
            end
            # Now, verify that we can run make regen
            Dir.chdir("build") do
                assert system("make", "regen")
                assert system("make")
            end
        end
    end

    def test_check_typekit_uptodate
        # Simulate an external library that define a particular type, which we
        # want to wrap with a typekit
        lib_prefix = File.join(prefix_directory, 'build_regen_library')
        FileUtils.mkdir_p prefix_directory
        FileUtils.rm_rf lib_prefix
        FileUtils.cp_r File.join(TEST_DATA_DIR, 'build_regen_library'), lib_prefix
        File.open(File.join(lib_prefix, 'build_regen_library.pc'), 'w') do |io|
            io.puts <<-PKGFILE
prefix=#{lib_prefix}

Name: build_regen_library
Description: fake library
Version: 0
Cflags: -I${prefix}/include -I${prefix}/include/project
            PKGFILE
        end
        ENV['PKG_CONFIG_PATH'] = "#{lib_prefix}:#{ENV['PKG_CONFIG_PATH']}"

        build_test_component "modules/build_regen_typekit", []

        in_wc do
            # Add a new type to test.h
            File.open('test.h', 'a') do |io|
                io.puts <<-NEWDEF
namespace Test {
    struct NewType {
        int field;
    };
}
                NEWDEF
            end

            # First check that the user is forbidden to go on with building
            Dir.chdir("build") do
                assert !system("make")
            end
            # Now, verify that we can run make regen
            Dir.chdir("build") do
                assert system("make", "regen")
                assert system("make")
            end

            registry = Typelib::Registry.import('.orogen/typekit/regen.tlb')
            assert registry.get("Test/NewType")
        end

        Dir.chdir(lib_prefix) do
            File.open('include/regen_lib.h', 'a') do |io|
                io.puts "struct RegenLibNewType { int field; };"
            end
        end
        in_wc do
            # First check that the user is forbidden to go on with building
            Dir.chdir("build") do
                assert !system("make")
            end
            # Now, verify that we can run make regen
            Dir.chdir("build") do
                assert system("make", "regen")
                assert system("make")
            end

            registry = Typelib::Registry.import('.orogen/typekit/regen.tlb')
            assert registry.get("RegenLibNewType")
        end
    end
end

