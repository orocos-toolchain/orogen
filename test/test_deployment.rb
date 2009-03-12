require 'orogen/test'

class TC_GenerationDeployment < Test::Unit::TestCase
    include Orocos::Generation::Test
    TEST_DATA_DIR = File.join( TEST_DIR, 'data' )

    def test_connpolicy
        policy = ConnPolicy.from_hash :type => :buffer, :lock_policy => :locked, :size => 10
        assert_equal(:buffer, policy.type)
        assert_equal(:locked, policy.lock_policy)
        assert_equal(false, policy.pull)
        assert_equal(10, policy.size)

        policy = ConnPolicy.from_hash(Hash.new)
        assert_equal(:data, policy.type)
        assert_equal(:lock_free, policy.lock_policy)
        assert_equal(false, policy.pull)
        assert_equal(0, policy.size)

        assert_raises(ArgumentError) { ConnPolicy.from_hash :type => :unknown }
        assert_raises(ArgumentError) { ConnPolicy.from_hash :lock_policy => :unknown }
        assert_raises(ArgumentError) { ConnPolicy.from_hash :type => :data, :size => 10 }
        assert_raises(ArgumentError) { ConnPolicy.from_hash :type => :data, :size => 0 }
        assert_raises(ArgumentError) { ConnPolicy.from_hash :type => :buffer }
        assert_raises(ArgumentError) { ConnPolicy.from_hash :type => :buffer, :size => 0 }
    end

    def test_data_driven_deployment(with_corba = false)
        build_test_component File.join(TEST_DATA_DIR, "deployment/data_triggered"), with_corba, "bin/data"

        # Check the resulting file
        in_prefix do
            assert_equal "U 2 4 6 8 10 ", File.read('data_trigger.txt')
        end
    end
end


