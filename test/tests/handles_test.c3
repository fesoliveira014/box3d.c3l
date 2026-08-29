module b3_test;

import b3;

fn void a_null_identifier_becomes_a_fault() @test {
    b3::WorldId world;

    if (catch err = world.checked()) {
        assert(err == b3::WORLD_LIMIT_REACHED);
        return;
    }
    assert(false, "a null world identifier must not be accepted");
}

fn void a_live_identifier_passes_through() @test {
    b3::BodyId body = { .index1 = 1, .world0 = 0, .generation = 1 };
    b3::BodyId got = body.checked()!!;
    assert(got.index1 == 1);
}

fn void the_checked_switch_reports_the_build() @test {
    $if $feat(BOX3D_CHECKED):
        assert(b3::checked_access());
    $else
        assert(!b3::checked_access());
    $endif
}
