module b3_test;

import b3;

fn void the_vendored_library_is_the_expected_version() @test {
    b3::Version version = b3::version();
    assert(version.major == 0);
    assert(version.minor == 2);
    assert(version.revision == 0);
}

fn void the_library_was_built_for_the_float_abi() @test {
    assert(!b3::is_double_precision());
    b3::require_float_abi()!!;
}

// c3c runs tests in alphabetical order by name across the whole target, so this name is chosen to
// sort after every world-creating test in the suite: it is a canary for a leak in any of them, not
// a check that no world has existed yet. A future rename should keep it sorting last, or it stops
// catching anything.
fn void zero_bytes_remain_once_earlier_worlds_are_destroyed() @test {
    assert(b3::byte_count() == 0);
}

fn void hashing_matches_the_c_implementation() @test {
    assert(b3::hash("box3d") == 254493957);
}

fn void the_default_length_unit_is_one_meter() @test {
    assert(b3::length_units_per_meter() == 1.0);
}
