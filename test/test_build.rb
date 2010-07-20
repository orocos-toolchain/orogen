$LOAD_PATH.unshift File.expand_path('../lib', File.dirname(__FILE__))
require 'orogen/test'

class TC_GenerationBuild < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

    def test_auto_regen_typekit
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
        puts ENV['PKG_CONFIG_PATH']

        build_test_component "modules/build_regen_typekit", false

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

            Dir.chdir("build") do
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
            Dir.chdir("build") do
                assert system("make")
            end

            registry = Typelib::Registry.import('.orogen/typekit/regen.tlb')
            assert registry.get("RegenLibNewType")
        end

    end

end

