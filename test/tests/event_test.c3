module b3_test;

import b3;
import std::math;

fn void the_event_types_match_the_c_layout() @test {
    assert(b3::SensorBeginTouchEvent::size == 16);
    assert(b3::SensorEndTouchEvent::size == 16);
    assert(b3::SensorEvents::size == 24);
    assert(b3::SensorEvents::alignment == 8);
    assert(b3::ContactBeginTouchEvent::size == 28);
    assert(b3::ContactEndTouchEvent::size == 28);
    assert(b3::ContactHitEvent::size == 72);
    assert(b3::ContactHitEvent::alignment == 8);
    assert(b3::ContactEvents::size == 40);
    assert(b3::BodyMoveEvent::size == 48);
    assert(b3::BodyMoveEvent::alignment == 8);
    assert(b3::BodyEvents::size == 16);
    assert(b3::JointEvent::size == 16);
    assert(b3::JointEvents::size == 16);
}

fn void two_bodies_that_collide_report_the_shapes_that_began_touching() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    b3::ShapeId ground_shape = ground.create_hull_shape(&ground_shape_def, &ground_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 5.0, 0.0};
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    shape_def.enable_contact_events = true;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    b3::ShapeId ball_shape = ball.create_sphere_shape(&shape_def, &sphere)!!;

    bool saw_begin;
    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);

        b3::ContactBeginTouchEvent[] begins = world.contact_begin_touch_events();
        if (begins.len == 0) continue;

        b3::ContactBeginTouchEvent touch = begins[0];
        bool ground_first = touch.shape_id_a.index1 == ground_shape.index1
                         && touch.shape_id_b.index1 == ball_shape.index1;
        bool ball_first = touch.shape_id_a.index1 == ball_shape.index1
                       && touch.shape_id_b.index1 == ground_shape.index1;
        assert(ground_first || ball_first, "a begin-touch event names the two shapes that touched");
        assert(touch.contact_id.is_valid(), "the contact behind a begin-touch event is live when it is reported");
        saw_begin = true;
        break;
    }

    assert(saw_begin, "a sphere dropped onto ground reports that it began touching");
}

fn void a_bouncing_sphere_reports_that_it_stopped_touching_the_ground() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    ground.create_hull_shape(&ground_shape_def, &ground_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 5.0, 0.0};
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    shape_def.enable_contact_events = true;
    shape_def.base_material.restitution = 0.6;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    ball.create_sphere_shape(&shape_def, &sphere)!!;

    int begin_total;
    int end_total;
    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
        begin_total += world.contact_begin_touch_events().len;
        end_total += world.contact_end_touch_events().len;
    }

    assert(begin_total >= 1, "the sphere must reach the ground");
    assert(end_total >= 1, "a sphere that bounces off the ground stops touching it");
}

fn void a_sphere_driven_into_the_ground_reports_the_speed_it_arrived_at() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    ground.create_hull_shape(&ground_shape_def, &ground_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.gravity_scale = 0.0;
    body_def.position = {0.0, 2.0, 0.0};
    body_def.linear_velocity = {0.0, -30.0, 0.0};
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    shape_def.enable_hit_events = true;
    shape_def.base_material.user_material_id = 7;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    ball.create_sphere_shape(&shape_def, &sphere)!!;

    bool saw_hit;
    for (int i = 0; i < 30; i++) {
        world.step(1.0f / 60.0f, 4);

        b3::ContactHitEvent[] hits = world.contact_hit_events();
        if (hits.len == 0) continue;

        b3::ContactHitEvent hit = hits[0];
        // The sphere is driven at 30 m/s into static ground, so it arrives at 30 m/s.
        assert(hit.approach_speed > 29.0 && hit.approach_speed < 31.0, "the hit reports the speed the shapes closed at");
        assert(math::abs(hit.normal.y) > 0.99 && math::abs(hit.normal.y) < 1.01, "a head-on vertical impact has a vertical normal");
        assert(math::abs(hit.normal.x) < 0.01 && math::abs(hit.normal.z) < 0.01);
        assert(hit.user_material_id_a == 7 || hit.user_material_id_b == 7, "one side of the hit carries the sphere's material");
        saw_hit = true;
        break;
    }

    assert(saw_hit, "an impact above the world's hit event threshold is reported");
}

fn void contact_events_stay_silent_when_no_shape_asks_for_them() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    ground.create_hull_shape(&ground_shape_def, &ground_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 2.0, 0.0};
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    ball.create_sphere_shape(&shape_def, &sphere)!!;

    int begin_total;
    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
        begin_total += world.contact_begin_touch_events().len;
    }

    assert(begin_total == 0, "a shape that did not ask for contact events reports none");
}

fn void a_world_with_nothing_in_it_reports_empty_contact_slices() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    world.step(1.0f / 60.0f, 4);

    assert(world.contact_begin_touch_events().len == 0);
    assert(world.contact_end_touch_events().len == 0);
    assert(world.contact_hit_events().len == 0);
}

fn void a_contact_reports_itself_invalid_once_one_of_its_bodies_is_gone() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    ground.create_hull_shape(&ground_shape_def, &ground_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 2.0, 0.0};
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    shape_def.enable_contact_events = true;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    ball.create_sphere_shape(&shape_def, &sphere)!!;

    b3::ContactId touched;
    bool captured;
    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);

        b3::ContactBeginTouchEvent[] begins = world.contact_begin_touch_events();
        if (begins.len == 0) continue;

        touched = begins[0].contact_id;
        captured = true;
        break;
    }

    assert(captured, "the sphere must reach the ground");
    assert(touched.is_valid());

    ball.destroy();
    assert(!touched.is_valid(), "a contact whose body is gone must not report itself valid");
}

fn void a_sensor_reports_the_shape_that_entered_and_left_it() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {1.5, 11.0, 0.0};
    b3::BodyId wall = world.create_body(&body_def)!!;
    b3::BoxHull wall_box = b3::make_box_hull(0.5, 10.0, 1.0);
    b3::ShapeDef wall_shape_def = b3::default_shape_def();
    wall_shape_def.enable_sensor_events = true;
    b3::ShapeId wall_shape = wall.create_hull_shape(&wall_shape_def, &wall_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.is_bullet = true;
    body_def.gravity_scale = 0.0;
    body_def.position = {7.39814, 4.0, 0.0};
    body_def.linear_velocity = {-20.0, 0.0, 0.0};
    b3::BodyId bullet = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.is_sensor = true;
    shape_def.enable_sensor_events = true;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.1};
    b3::ShapeId bullet_shape = bullet.create_sphere_shape(&shape_def, &sphere)!!;

    int begin_total;
    int end_total;
    for (int i = 0; i < 200; i++) {
        world.step(1.0f / 60.0f, 4);

        foreach (entered : world.sensor_begin_touch_events()) {
            begin_total++;
            assert(entered.sensor_shape_id.index1 == bullet_shape.index1, "the sensor is the shape that declared itself one");
            assert(entered.visitor_shape_id.index1 == wall_shape.index1, "the visitor is the shape that passed through it");
        }
        foreach (left : world.sensor_end_touch_events()) {
            end_total++;
            assert(left.sensor_shape_id.index1 == bullet_shape.index1);
            assert(left.visitor_shape_id.index1 == wall_shape.index1);
        }

        if (bullet.position().x < -1.0f) break;
    }

    assert(begin_total == 1, "a sensor swept once through a wall enters it once");
    assert(end_total == 1, "and leaves it once");
}

fn void a_visitor_that_did_not_ask_for_sensor_events_is_never_reported() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {1.5, 11.0, 0.0};
    b3::BodyId wall = world.create_body(&body_def)!!;
    b3::BoxHull wall_box = b3::make_box_hull(0.5, 10.0, 1.0);
    b3::ShapeDef wall_shape_def = b3::default_shape_def();
    wall.create_hull_shape(&wall_shape_def, &wall_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.is_bullet = true;
    body_def.gravity_scale = 0.0;
    body_def.position = {7.39814, 4.0, 0.0};
    body_def.linear_velocity = {-20.0, 0.0, 0.0};
    b3::BodyId bullet = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.is_sensor = true;
    shape_def.enable_sensor_events = true;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.1};
    bullet.create_sphere_shape(&shape_def, &sphere)!!;

    int begin_total;
    for (int i = 0; i < 200; i++) {
        world.step(1.0f / 60.0f, 4);
        begin_total += world.sensor_begin_touch_events().len;
        if (bullet.position().x < -1.0f) break;
    }

    assert(begin_total == 0, "a sensor overlap is reported only when the visitor asked for sensor events too");
}

fn void a_moving_body_reports_the_transform_it_reached() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    ground.create_hull_shape(&ground_shape_def, &ground_box.base)!!;

    int marker = 42;
    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 2.0, 0.0};
    body_def.user_data = &marker;
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    ball.create_sphere_shape(&shape_def, &sphere)!!;

    bool saw_move;
    for (int i = 0; i < 30; i++) {
        world.step(1.0f / 60.0f, 4);

        foreach (moved : world.body_move_events()) {
            if (moved.body_id.index1 != ball.index1) continue;

            b3::WorldTransform pose = ball.transform();
            assert(moved.transform.p.x == pose.p.x, "the move event carries the pose the body ended the step at");
            assert(moved.transform.p.y == pose.p.y);
            assert(moved.transform.p.z == pose.p.z);
            assert(moved.user_data == &marker, "the move event carries the body's own user data");
            saw_move = true;
        }
    }

    assert(saw_move, "a falling body reports that it moved");
}

fn void a_body_that_settles_says_so_once_and_then_goes_quiet() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    ground.create_hull_shape(&ground_shape_def, &ground_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 2.0, 0.0};
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    ball.create_sphere_shape(&shape_def, &sphere)!!;

    int fell_asleep_total;
    for (int i = 0; i < 400; i++) {
        world.step(1.0f / 60.0f, 4);

        foreach (moved : world.body_move_events()) {
            if (moved.fell_asleep) fell_asleep_total++;
        }
    }

    assert(fell_asleep_total == 1, "a body that comes to rest reports falling asleep exactly once");
    assert(world.body_move_events().len == 0, "a sleeping body reports no movement");
}

fn void a_world_with_nothing_moving_reports_empty_sensor_and_move_slices() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    world.step(1.0f / 60.0f, 4);

    assert(world.sensor_begin_touch_events().len == 0);
    assert(world.sensor_end_touch_events().len == 0);
    assert(world.body_move_events().len == 0);
}

int pre_solve_calls;
int pre_solve_context_calls;

fn bool count_pre_solve(b3::ShapeId shape_id_a, b3::ShapeId shape_id_b, b3::Pos point, b3::Vec3 normal, void* context) {
    pre_solve_calls++;
    int* seen = (int*)context;
    (*seen)++;
    return true;
}

fn bool refuse_pre_solve(b3::ShapeId shape_id_a, b3::ShapeId shape_id_b, b3::Pos point, b3::Vec3 normal, void* context) {
    return false;
}

int custom_filter_calls;

fn bool refuse_pair(b3::ShapeId shape_id_a, b3::ShapeId shape_id_b, void* context) {
    custom_filter_calls++;
    return false;
}

fn void a_world_without_joints_reports_no_joint_events() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    world.step(1.0f / 60.0f, 4);

    assert(world.joint_events().len == 0);
}

fn void a_joint_identifier_that_names_nothing_is_not_valid() @test {
    b3::JointId nothing;

    assert(!nothing.is_valid(), "a zeroed joint identifier names no joint");
}

fn void a_pre_solve_callback_sees_the_contact_and_the_context_it_was_given() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    int seen;
    pre_solve_calls = 0;
    world.set_pre_solve_callback(&count_pre_solve, &seen);

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    ground_shape_def.enable_pre_solve_events = true;
    ground.create_hull_shape(&ground_shape_def, &ground_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 1.0, 0.0};
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    shape_def.enable_pre_solve_events = true;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    ball.create_sphere_shape(&shape_def, &sphere)!!;

    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    assert(pre_solve_calls >= 1, "a contact between two shapes that asked for pre-solve events reaches the callback");
    assert(seen == pre_solve_calls, "the callback receives the context it was registered with");
}

fn void a_pre_solve_callback_that_refuses_a_contact_lets_the_body_fall_through() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    world.set_pre_solve_callback(&refuse_pre_solve, null);

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    ground_shape_def.enable_pre_solve_events = true;
    ground.create_hull_shape(&ground_shape_def, &ground_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 1.0, 0.0};
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    shape_def.enable_pre_solve_events = true;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    ball.create_sphere_shape(&shape_def, &sphere)!!;

    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    assert(ball.position().y < -5.0f, "a contact the callback refuses does not hold the body up");
}

fn void a_custom_filter_that_refuses_a_pair_lets_the_body_fall_through() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    custom_filter_calls = 0;
    world.set_custom_filter_callback(&refuse_pair, null);

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    ground_shape_def.enable_custom_filtering = true;
    ground.create_hull_shape(&ground_shape_def, &ground_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 1.0, 0.0};
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    shape_def.enable_custom_filtering = true;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    ball.create_sphere_shape(&shape_def, &sphere)!!;

    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    assert(custom_filter_calls >= 1, "a pair of shapes that asked for custom filtering reaches the callback");
    assert(ball.position().y < -5.0f, "a pair the filter refuses never collides");
}
