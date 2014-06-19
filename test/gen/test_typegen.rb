require 'orogen/gen/test'

class TC_GenerationTypegen < Minitest::Test
    def test_generate_and_install
        build_typegen "simple", "modules/typekit_simple/simple.h", []
    end

    def test_check_uptodate
        build_typegen "simple", ["modules/typekit_simple/simple.h"], []

        in_wc("typekit_output") do
            # Touch the orogen file
            FileUtils.touch File.join("..", "types", "simple.h")

            # First check that the user is forbidden to go on with building
            Dir.chdir("build") do
                assert !call_make
            end
            # Now, verify that we can run make regen and build fine
            Dir.chdir("build") do
                assert call_make('regen')
                assert call_make
            end
        end
    end
end

