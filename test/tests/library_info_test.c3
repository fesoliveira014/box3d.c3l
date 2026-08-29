module b3_test;

import b3;

fn void the_vendored_library_is_the_expected_version() @test {
    b3::Version version = b3::get_version();
    assert(version.major == 0);
    assert(version.minor == 2);
    assert(version.revision == 0);
}

fn void the_library_was_built_for_the_float_abi() @test {
    assert(!b3::is_double_precision());
    b3::assert_float_abi()!!;
}

fn void nothing_is_allocated_before_a_world_exists() @test {
    assert(b3::byte_count() == 0);
}

fn void hashing_matches_the_c_implementation() @test {
    assert(b3::hash("box3d") == 254493957);
}

fn void the_default_length_unit_is_one_meter() @test {
    assert(b3::length_units_per_meter() == 1.0);
}
