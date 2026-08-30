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

fn void a_body_lives_and_dies_inside_its_world() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    b3::BodyId body = world.create_body(&bd)!!;

    assert(body.is_valid());
    assert(body.type() == b3::BodyType.STATIC);
    assert(body.is_enabled());

    body.destroy();
    assert(!body.is_valid(), "a destroyed body must not report itself valid");
}

fn void a_zeroed_body_definition_is_refused() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef zeroed;
    if (catch err = world.create_body(&zeroed)) {
        assert(err == b3::INVALID_DEFINITION);
        return;
    }
    assert(false, "box3d accepts a zeroed body definition; the binding must not");
}

fn void a_body_knows_which_world_holds_it() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    b3::BodyId body = world.create_body(&bd)!!;

    assert(body.world().index1 == world.index1);
    assert(body.world().generation == world.generation);
}

fn void a_disabled_body_can_be_enabled_again() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&bd)!!;

    body.disable();
    assert(!body.is_enabled());
    body.enable();
    assert(body.is_enabled());

    body.set_type(b3::BodyType.KINEMATIC);
    assert(body.type() == b3::BodyType.KINEMATIC);
}
