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
    assert(!def.is_bullet);
    assert(def.is_enabled);
    assert(!def.allow_fast_rotation);
    assert(def.enable_contact_recycling);
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

    for (int i = 0; i < 60; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    float vy = body.linear_velocity().y;
    assert(vy < -9.99 && vy > -10.01, "one second under ten metres per second squared");

    float dropped = 100.0f - body.position().y;
    assert(dropped > 4.9 && dropped < 5.1, "and roughly five metres of fall");
}

fn void gravity_scale_changes_the_fall() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&bd)!!;
    body.set_gravity_scale(0.5);
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

fn void a_body_name_round_trips() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    b3::BodyId body = world.create_body(&bd)!!;

    assert(body.name().str_view() == "", "an unnamed body reports an empty name, never null");

    body.set_name("anvil");
    assert(body.name().str_view() == "anvil");
}

fn void user_data_round_trips_on_a_body() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    b3::BodyId body = world.create_body(&bd)!!;

    int sentinel = 7;
    body.set_user_data(&sentinel);
    assert(body.user_data() == &sentinel);
}

fn void motion_locks_round_trip() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&bd)!!;

    b3::MotionLocks locks = { .linear_x = true, .angular_z = true };
    body.set_motion_locks(locks);

    b3::MotionLocks got = body.motion_locks();
    assert(got.linear_x);
    assert(!got.linear_y);
    assert(!got.linear_z);
    assert(!got.angular_x);
    assert(!got.angular_y);
    assert(got.angular_z);
}

fn void each_body_flag_moves_only_itself() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&bd)!!;

    assert(body.is_sleep_enabled());
    assert(!body.is_bullet());
    assert(!body.is_fast_rotation_allowed());
    assert(body.is_contact_recycling_enabled());

    body.set_bullet(true);
    assert(body.is_bullet());
    assert(body.is_sleep_enabled(), "setting bullet must not touch sleep");
    assert(!body.is_fast_rotation_allowed(), "setting bullet must not touch fast rotation");

    body.allow_fast_rotation(true);
    assert(body.is_fast_rotation_allowed());
    assert(body.is_bullet(), "allowing fast rotation must not clear bullet");

    body.enable_sleep(false);
    assert(!body.is_sleep_enabled());
    assert(body.is_bullet(), "disabling sleep must not clear bullet");

    body.set_sleep_threshold(0.25);
    assert(body.sleep_threshold() == 0.25);

    body.enable_contact_recycling(false);
    assert(!body.is_contact_recycling_enabled());
    assert(body.is_bullet(), "disabling contact recycling must not clear bullet");
    assert(body.is_fast_rotation_allowed(), "disabling contact recycling must not clear fast rotation");

    // box3d ships no reader for this flag, so there is nothing to assert beyond the call
    // succeeding — b3Body_EnableHitEvents has no b3Body_AreHitEventsEnabled counterpart.
    body.enable_hit_events(true);
}

fn void a_sleeping_body_wakes_when_told() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&bd)!!;

    assert(body.is_awake());
    body.set_awake(false);
    assert(!body.is_awake());
    body.set_awake(true);
    assert(body.is_awake());
}

fn void a_spinning_body_reports_the_velocity_at_every_point_it_carries() @test {
    b3::WorldDef wd = b3::default_world_def();
    wd.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.type = b3::BodyType.DYNAMIC;
    bd.position = (b3::Pos){ 10.0, 0.0, 0.0 };
    b3::BodyId body = world.create_body(&bd)!!;

    b3::MassData md = {
        .mass = 1.0,
        .center = { 0.0, 0.0, 0.0 },
        .inertia = { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } },
    };
    body.set_mass_data(md);

    body.set_linear_velocity((b3::Vec3){ 1.0, 0.0, 0.0 });
    body.set_angular_velocity((b3::Vec3){ 0.0, 0.0, 2.0 });

    b3::Vec3 linear = body.linear_velocity();
    b3::Vec3 angular = body.angular_velocity();
    assert(linear.x == 1.0 && linear.y == 0.0 && linear.z == 0.0);
    assert(angular.x == 0.0 && angular.y == 0.0 && angular.z == 2.0);

    b3::Vec3 local_v = body.local_point_velocity((b3::Vec3){ 1.0, 0.0, 0.0 });
    assert(local_v.x > 0.99 && local_v.x < 1.01);
    assert(local_v.y > 1.99 && local_v.y < 2.01);
    assert(local_v.z > -0.01 && local_v.z < 0.01);

    b3::Vec3 world_v = body.world_point_velocity((b3::Pos){ 1.0, 0.0, 0.0 });
    assert(world_v.x > 0.99 && world_v.x < 1.01);
    assert(world_v.y > -18.01 && world_v.y < -17.99);
    assert(world_v.z > -0.01 && world_v.z < 0.01);

    // 90 degrees about z: sin(45 deg) = cos(45 deg) = sqrt(2)/2
    b3::Quat quarter_turn = { .i = 0.0, .j = 0.0, .k = 0.70710678, .l = 0.70710678 };
    body.set_transform(body.position(), quarter_turn);

    b3::Vec3 turned_world = body.world_vector((b3::Vec3){ 1.0, 0.0, 0.0 });
    assert(turned_world.x > -0.01 && turned_world.x < 0.01);
    assert(turned_world.y > 0.99 && turned_world.y < 1.01);
    assert(turned_world.z > -0.01 && turned_world.z < 0.01);

    b3::Vec3 turned_local = body.local_vector((b3::Vec3){ 0.0, 1.0, 0.0 });
    assert(turned_local.x > 0.99 && turned_local.x < 1.01);
    assert(turned_local.y > -0.01 && turned_local.y < 0.01);
    assert(turned_local.z > -0.01 && turned_local.z < 0.01);
}

fn void force_and_torque_accumulate_over_a_step_the_way_the_arithmetic_says() @test {
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

    float dt = 1.0f / 60.0f;

    body.apply_force_to_center((b3::Vec3){ 4.0, 0.0, 0.0 }, true);
    world.step(dt, 4);
    float vx = body.linear_velocity().x;
    assert(vx > 0.032 && vx < 0.035, "a force of four on a mass of two accelerates at two");
    body.set_linear_velocity((b3::Vec3){ 0.0, 0.0, 0.0 });

    body.apply_torque((b3::Vec3){ 0.0, 0.0, 2.0 }, true);
    world.step(dt, 4);
    float wz = body.angular_velocity().z;
    assert(wz > 0.032 && wz < 0.035, "a torque of two about a unit inertia accelerates at two");
    body.set_angular_velocity((b3::Vec3){ 0.0, 0.0, 0.0 });

    body.apply_angular_impulse((b3::Vec3){ 0.0, 0.0, 1.0 }, true);
    float wz2 = body.angular_velocity().z;
    assert(wz2 > 0.99 && wz2 < 1.01, "an angular impulse applies immediately, with no step");
    body.set_angular_velocity((b3::Vec3){ 0.0, 0.0, 0.0 });

    body.apply_linear_impulse((b3::Vec3){ 0.0, 4.0, 0.0 }, (b3::Pos){ 1.0, 0.0, 0.0 }, true);
    float vy = body.linear_velocity().y;
    float wz3 = body.angular_velocity().z;
    assert(vy > 1.99 && vy < 2.01, "an impulse of four on a mass of two gives a speed of two");
    assert(wz3 > 3.9 && wz3 < 4.1, "the off-center point makes the impulse apply torque too");
    body.set_linear_velocity((b3::Vec3){ 0.0, 0.0, 0.0 });
    body.set_angular_velocity((b3::Vec3){ 0.0, 0.0, 0.0 });

    body.apply_force((b3::Vec3){ 0.0, 0.0, 6.0 }, (b3::Pos){ 1.0, 0.0, 0.0 }, true);
    world.step(dt, 4);
    float vz = body.linear_velocity().z;
    float wy = body.angular_velocity().y;
    assert(vz > 0.045 && vz < 0.055, "the force at the center gives linear acceleration of three");
    assert(wy > -0.11 && wy < -0.09, "the same force off-center gives angular acceleration through its torque");
}

fn void mass_data_reads_back_through_every_mass_accessor() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.type = b3::BodyType.DYNAMIC;
    bd.position = (b3::Pos){ 10.0, 0.0, 0.0 };
    b3::BodyId body = world.create_body(&bd)!!;

    b3::MassData md = {
        .mass = 4.0,
        .center = { 0.5, 0.0, 0.0 },
        .inertia = { { 2.0, 0.0, 0.0 }, { 0.0, 4.0, 0.0 }, { 0.0, 0.0, 8.0 } },
    };
    body.set_mass_data(md);

    assert(body.mass() == 4.0);
    assert(body.inverse_mass() == 0.25);

    b3::MassData got = body.mass_data();
    assert(got.mass == 4.0);
    assert(got.center.x == 0.5 && got.center.y == 0.0 && got.center.z == 0.0);
    assert(got.inertia[0].x == 2.0 && got.inertia[0].y == 0.0 && got.inertia[0].z == 0.0);
    assert(got.inertia[1].x == 0.0 && got.inertia[1].y == 4.0 && got.inertia[1].z == 0.0);
    assert(got.inertia[2].x == 0.0 && got.inertia[2].y == 0.0 && got.inertia[2].z == 8.0);

    b3::Matrix3 local_inertia = body.local_rotational_inertia();
    assert(local_inertia[0].x == 2.0 && local_inertia[1].y == 4.0 && local_inertia[2].z == 8.0);

    // The inertia diagonal is chosen as powers of two so its reciprocal is exact.
    b3::Matrix3 world_inv_inertia = body.world_inverse_rotational_inertia();
    assert(world_inv_inertia[0].x == 0.5);
    assert(world_inv_inertia[1].y == 0.25);
    assert(world_inv_inertia[2].z == 0.125);

    b3::Vec3 local_c = body.local_center();
    assert(local_c.x == 0.5 && local_c.y == 0.0 && local_c.z == 0.0);

    b3::Pos world_c = body.world_center();
    assert(world_c.x == 10.5 && world_c.y == 0.0 && world_c.z == 0.0);

    body.apply_mass_from_shapes();
    assert(body.mass() == 0.0, "a body with no shapes recomputes to no mass");
    b3::Vec3 reset_center = body.local_center();
    assert(reset_center.x == 0.0 && reset_center.y == 0.0 && reset_center.z == 0.0);
}

fn void a_kinematic_body_driven_to_a_target_arrives_there() @test {
    b3::WorldDef wd = b3::default_world_def();
    b3::WorldId world = b3::create_world(&wd)!!;
    defer world.destroy();

    b3::BodyDef bd = b3::default_body_def();
    bd.type = b3::BodyType.KINEMATIC;
    b3::BodyId body = world.create_body(&bd)!!;

    b3::WorldTransform target = {
        .p = { 1.0, 0.0, 0.0 },
        .q = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 },
    };
    body.set_target_transform(target, 1.0f / 60.0f, true);

    b3::Vec3 linear = body.linear_velocity();
    b3::Vec3 angular = body.angular_velocity();
    assert(linear.x > 59.9 && linear.x < 60.1, "the velocity that closes one metre in one sixtieth of a second is sixty");
    assert(linear.y > -0.01 && linear.y < 0.01);
    assert(linear.z > -0.01 && linear.z < 0.01);
    assert(angular.x > -0.01 && angular.x < 0.01);
    assert(angular.y > -0.01 && angular.y < 0.01);
    assert(angular.z > -0.01 && angular.z < 0.01);

    world.step(1.0f / 60.0f, 4);

    b3::WorldTransform got = body.transform();
    assert(got.p.x > 0.99 && got.p.x < 1.01);
    assert(got.q.l > 0.99 && got.q.l < 1.01);
    assert(body.position().x > 0.99 && body.position().x < 1.01, "position() must agree with transform().p");
    assert(body.rotation().l > 0.99 && body.rotation().l < 1.01, "rotation() must agree with transform().q");

    b3::Aabb box = body.compute_aabb();
    assert(box.lower_bound.x > 0.99 && box.lower_bound.x < 1.01);
    assert(box.lower_bound.y > -0.01 && box.lower_bound.y < 0.01);
    assert(box.lower_bound.z > -0.01 && box.lower_bound.z < 0.01);
    assert(box.upper_bound.x == box.lower_bound.x, "a shape-free body's AABB is degenerate: both bounds equal its position");
    assert(box.upper_bound.y == box.lower_bound.y);
    assert(box.upper_bound.z == box.lower_bound.z);
}

fn void a_body_reports_the_shapes_attached_to_it() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere first = {.center = {-1.0, 0.0, 0.0}, .radius = 0.5};
    b3::Sphere second = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    b3::Sphere third = {.center = {1.0, 0.0, 0.0}, .radius = 0.5};
    body.create_sphere_shape(&shape_def, &first)!!;
    body.create_sphere_shape(&shape_def, &second)!!;
    body.create_sphere_shape(&shape_def, &third)!!;

    assert(body.shape_count() == 3, "three spheres were attached");

    b3::ShapeId[8] roomy;
    b3::ShapeId[] shapes = body.shapes(roomy[..]);
    assert(shapes.len == 3, "and all three are reported");
    assert(shapes[0].is_valid() && shapes[1].is_valid() && shapes[2].is_valid(),
        "each reported shape is live");

    b3::ShapeId[2] cramped;
    b3::ShapeId[] some = body.shapes(cramped[..]);
    assert(some.len == 2, "a short buffer is filled to its length and no further");
}
