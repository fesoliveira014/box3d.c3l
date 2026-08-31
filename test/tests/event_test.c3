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
