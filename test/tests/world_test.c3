module b3_test;

import b3;

fn void the_world_types_match_the_c_layout() @test {
    assert(b3::Capacity::size == 20);
    assert(b3::WorldDef::size == 144);
    assert(b3::WorldDef::alignment == 8);
    assert(b3::Profile::size == 92);
    assert(b3::Counters::size == 200);
}

fn void the_default_definition_carries_the_values_the_c_side_sets() @test {
    b3::WorldDef def = b3::default_world_def();

    assert(def.gravity.x == 0.0);
    assert(def.gravity.y == -10.0);
    assert(def.gravity.z == 0.0);
    assert(def.restitution_threshold == 1.0);
    assert(def.hit_event_threshold == 1.0);
    assert(def.contact_hertz == 30.0);
    assert(def.contact_damping_ratio == 10.0);
    assert(def.contact_speed == 3.0);
    assert(def.maximum_linear_speed == 400.0);
    assert(def.enable_sleep);
    assert(def.enable_continuous);
}

fn void a_zeroed_definition_differs_from_a_default_one() @test {
    b3::WorldDef zeroed;
    b3::WorldDef def = b3::default_world_def();

    assert(zeroed.internal_value != def.internal_value);
}
