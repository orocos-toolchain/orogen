# frozen_string_literal: true

require "orogen/gen/test"

module OroGen
    module Gen
        describe RTT_CPP do
            it "loads and caches loaded ERB templates" do
                assert_raises(ArgumentError) do
                    RTT_CPP.load_template("does", "not", "exist")
                end

                erb = RTT_CPP.load_template("typekit", "Plugin.cpp")
                assert_kind_of(ERB, erb)

                assert_same(erb, RTT_CPP.load_template("typekit", "Plugin.cpp"))

                other_erb = RTT_CPP.load_template("CMakeLists.txt")
                refute_same(other_erb, erb)
            end

            it "saves result files into the user part of the generation" do
                create_wc "base"
                assert_raises(ArgumentError) { RTT_CPP.save_user }

                in_wc do
                    RTT_CPP.save_user "test.cpp", "blabla"
                end

                target_file = File.join(working_directory, "test.cpp")
                assert_equal("blabla", File.read(target_file))

                in_wc do
                    RTT_CPP.save_user "subdir", "test.cpp", "bloblo"
                end

                target_file = File.join(working_directory, "subdir", "test.cpp")
                assert_equal("bloblo", File.read(target_file))
            end

            it "saves result files into the 'automatic' part of the generation" do
                create_wc "base"
                assert_raises(ArgumentError) { RTT_CPP.save_automatic }

                in_wc do
                    RTT_CPP.save_automatic "test.cpp", "blabla"
                end

                target_file = File.join(working_directory, RTT_CPP::AUTOMATIC_AREA_NAME,
                                        "test.cpp")
                assert_equal("blabla", File.read(target_file))

                in_wc do
                    RTT_CPP.save_automatic "subdir", "test.cpp", "bloblo"
                end

                target_file = File.join(working_directory, RTT_CPP::AUTOMATIC_AREA_NAME,
                                        "subdir", "test.cpp")
                assert_equal("bloblo", File.read(target_file))
            end

            it "produces C++ code to change namespaces with proper indentation" do
                assert_equal "    }\n", RTT_CPP.adapt_namespace("/A/B", "/A")
                assert_equal "        }\n    }\n    namespace D {\n",
                             RTT_CPP.adapt_namespace("/A/B/C", "/A/D")
                assert_equal "", RTT_CPP.adapt_namespace("/A/B/C", "/A/B/C")
            end

            describe RTT_CPP::JobServer do
                it "successfully gets and puts tokens" do
                    job_server = RTT_CPP::JobServer.standalone(2)
                    10.times do
                        job_server.get
                        job_server.put
                    end
                end

                it "limits the number of concurrent 'get' "\
                   "to the count of tokens available" do
                    job_server = RTT_CPP::JobServer.standalone(2)
                    verify_get_limit(job_server, 2)
                end

                it "can be initialized with pre-allocated I/O" do
                    r, w = IO.pipe
                    job_server = RTT_CPP::JobServer.from_fds(r.fileno, w.fileno)
                    w.write("  ")
                    verify_get_limit(job_server, 2)
                end

                def verify_get_limit(job_server, count)
                    get_thread = Thread.new do
                        (count + 1).times { job_server.get }
                    end

                    Thread.pass until get_thread.status == "sleep"
                    job_server.put
                    get_thread.join
                end
            end
        end
    end
end
