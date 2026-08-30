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

fn void a_body_with_mass_falls_under_gravity() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.type = b3::BodyType.DYNAMIC;
    bd.position = (b3::Pos){ 0.0, 100.0, 0.0 };
    b3::BodyId body = world.create_body(&bd)!!;

    assert(body.mass() == 0.0, "a body carrying no shape has no mass");

    b3::MassData md = {
        .mass = 1.0,
        .center = { 0.0, 0.0, 0.0 },
        .inertia = { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } },
    };
    body.set_mass_data(md);
    assert(body.mass() == 1.0);
    assert(body.inverse_mass() == 1.0);

    for (int i = 0; i < 60; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    float vy = body.linear_velocity().y;
    assert(vy < -9.99 && vy > -10.01, "one second under ten metres per second squared");

    float dropped = 100.0f - (float)body.position().y;
    assert(dropped > 4.9 && dropped < 5.1, "and roughly five metres of fall");
}

fn void gravity_scale_changes_the_fall() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.type = b3::BodyType.DYNAMIC;
    bd.gravity_scale = 0.5;
    b3::BodyId body = world.create_body(&bd)!!;
    assert(body.gravity_scale() == 0.5);

    b3::MassData md = {
        .mass = 1.0,
        .center = { 0.0, 0.0, 0.0 },
        .inertia = { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } },
    };
    body.set_mass_data(md);

    for (int i = 0; i < 60; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    float vy = body.linear_velocity().y;
    assert(vy < -4.99 && vy > -5.01, "half gravity gives half the speed");
}

fn void an_impulse_moves_a_body_the_way_the_arithmetic_says() @test {
    b3::WorldDef wd = b3::default_world_def();
    wd.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&bd)!!;

    b3::MassData md = {
        .mass = 2.0,
        .center = { 0.0, 0.0, 0.0 },
        .inertia = { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } },
    };
    body.set_mass_data(md);

    b3::Vec3 impulse = { 4.0, 0.0, 0.0 };
    body.apply_linear_impulse_to_center(impulse, true);

    b3::Vec3 v = body.linear_velocity();
    assert(v.x > 1.99 && v.x < 2.01, "an impulse of four on a mass of two gives a speed of two");
}

fn void a_transform_round_trips() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    b3::BodyId body = world.create_body(&bd)!!;

    b3::Pos where = { 1.5, 2.5, -3.5 };
    b3::Quat identity = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 };
    body.set_transform(where, identity);

    b3::Pos got = body.position();
    assert(got.x == 1.5 && got.y == 2.5 && got.z == -3.5);
    assert(body.rotation().l == 1.0);
}

fn void local_and_world_coordinates_are_inverses() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.position = (b3::Pos){ 10.0, 0.0, 0.0 };
    b3::BodyId body = world.create_body(&bd)!!;

    b3::Vec3 local = { 1.0, 2.0, 3.0 };
    b3::Pos in_world = body.world_point(local);
    assert(in_world.x == 11.0 && in_world.y == 2.0 && in_world.z == 3.0);

    b3::Vec3 back = body.local_point(in_world);
    assert(back.x == 1.0 && back.y == 2.0 && back.z == 3.0);
}

fn void damping_round_trips() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&bd)!!;

    body.set_linear_damping(0.25);
    body.set_angular_damping(0.75);
    assert(body.linear_damping() == 0.25);
    assert(body.angular_damping() == 0.75);
}
