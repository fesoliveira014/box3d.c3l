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

fn void the_constraint_graph_colours_are_refused_outside_the_table() @test {
    draw::HexColor first = draw::graph_color(0)!!;
    draw::HexColor overflow = draw::graph_color(draw::GRAPH_COLOR_COUNT - 1)!!;

    fault under;
    fault over;
    if (catch err = draw::graph_color(-1)) under = err;
    if (catch err = draw::graph_color(draw::GRAPH_COLOR_COUNT)) over = err;

    assert(first == 0xFF0000 && overflow == 0xC0C0C0, "the table runs from red to the silver overflow slot");
    assert(under == b3::INDEX_OUT_OF_RANGE, "box3d reads a negative index and answers black");
    assert(over == b3::INDEX_OUT_OF_RANGE, "and reads past the table for a value that is neither a colour nor the same twice");
}
