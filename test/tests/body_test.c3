module b3_test;

import b3;

fn void the_body_types_match_the_c_layout() @test {
    assert(b3::BodyType::size == 4);
    assert(b3::MotionLocks::size == 6);
    assert(b3::MotionLocks::alignment == 1);
    assert(b3::MassData::size == 52);
    assert(b3::BodyDef::size == 104);
    assert(b3::BodyDef::alignment == 8);
}

fn void the_body_type_values_match_the_c_enum() @test {
    assert((int)b3::BodyType.STATIC == 0);
    assert((int)b3::BodyType.KINEMATIC == 1);
    assert((int)b3::BodyType.DYNAMIC == 2);
}

fn void the_default_body_definition_carries_the_values_the_c_side_sets() @test {
    b3::BodyDef def = b3::default_body_def();

    assert(def.type == b3::BodyType.STATIC);
    assert(def.gravity_scale == 1.0);
    assert(def.sleep_threshold == (float)0.05);
    assert(def.linear_damping == 0.0);
    assert(def.angular_damping == 0.0);
    assert(def.enable_sleep);
    assert(def.is_awake);
    assert(def.is_enabled);
}
