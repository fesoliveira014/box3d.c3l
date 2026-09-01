module b3_test;

import b3;
import std::math;

fn b3::WorldId a_world_with_a_ball_resting_on_ground(b3::BodyId* out_ball, b3::ShapeId* out_ball_shape) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    ground.create_hull_shape(&ground_shape_def, &ground_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 0.5, 0.0};
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    b3::ShapeId ball_shape = ball.create_sphere_shape(&shape_def, &sphere)!!;

    for (int i = 0; i < 60; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    *out_ball = ball;
    *out_ball_shape = ball_shape;
    return world;
}

fn void a_resting_body_reports_its_touching_contact() @test {
    b3::BodyId ball;
    b3::ShapeId ball_shape;
    b3::WorldId world = a_world_with_a_ball_resting_on_ground(&ball, &ball_shape);
    defer world.destroy();

    b3::ContactData[8] storage;
    b3::ContactData[] contacts = ball.contact_data(storage[..]);
    assert(contacts.len == 1, "the resting ball touches the ground and nothing else");

    b3::ContactData contact = contacts[0];
    assert(contact.contact_id.is_valid(), "the reported contact is live");

    bool names_the_ball = contact.shape_id_a.index1 == ball_shape.index1
                       || contact.shape_id_b.index1 == ball_shape.index1;
    assert(names_the_ball, "and names the ball's shape as one of its two");
}

fn void contact_capacity_is_an_upper_bound_on_what_is_written() @test {
    b3::BodyId ball;
    b3::ShapeId ball_shape;
    b3::WorldId world = a_world_with_a_ball_resting_on_ground(&ball, &ball_shape);
    defer world.destroy();

    int capacity = ball.contact_capacity();
    b3::ContactData[8] storage;
    b3::ContactData[] contacts = ball.contact_data(storage[..]);

    assert(capacity >= 1, "the resting ball has at least one contact to report");
    assert((usz)capacity >= contacts.len, "capacity never under-reports what contact_data writes");

    b3::ContactData[] nothing = ball.contact_data(storage[:0]);
    assert(nothing.len == 0, "a zero-length buffer is filled with nothing");
}

fn void a_contact_carries_its_manifold_geometry() @test {
    b3::BodyId ball;
    b3::ShapeId ball_shape;
    b3::WorldId world = a_world_with_a_ball_resting_on_ground(&ball, &ball_shape);
    defer world.destroy();

    b3::ContactData[8] storage;
    b3::ContactData[] contacts = ball.contact_data(storage[..]);
    assert(contacts.len == 1, "the resting ball reports its one contact");

    b3::ContactData contact = contacts[0];
    b3::Manifold[] manifolds = contact.contact_manifolds();
    assert(manifolds.len == (usz)contact.manifold_count, "the slice spans exactly the reported count");
    assert(manifolds.len >= 1, "a touching contact has geometry");

    b3::Manifold manifold = manifolds[0];
    float normal_length = math::sqrt(
      manifold.normal.x * manifold.normal.x
      + manifold.normal.y * manifold.normal.y
      + manifold.normal.z * manifold.normal.z);
    assert(normal_length > 0.999f && normal_length < 1.001f, "the manifold normal is a unit vector");
    assert(manifold.normal.y > 0.9f || manifold.normal.y < -0.9f,
        "and points along the axis separating a ball from flat ground");

    b3::ManifoldPoint[] points = manifold.contact_points();
    assert(points.len == (usz)manifold.point_count, "the point slice spans exactly point_count");
    assert(points.len >= 1 && points.len <= 4, "a manifold carries between one and four points");
    assert(points[0].separation > -0.01f && points[0].separation < 0.01f,
        "a resting sphere sits within a hair of the ground, neither overlapping nor floating");
}

fn void a_shape_reports_the_same_contact_as_its_body() @test {
    b3::BodyId ball;
    b3::ShapeId ball_shape;
    b3::WorldId world = a_world_with_a_ball_resting_on_ground(&ball, &ball_shape);
    defer world.destroy();

    b3::ContactData[8] body_storage;
    b3::ContactData[] from_body = ball.contact_data(body_storage[..]);

    b3::ContactData[8] shape_storage;
    b3::ContactData[] from_shape = ball_shape.contact_data(shape_storage[..]);

    assert(from_body.len == 1, "the ball has one contact");
    assert(from_shape.len == from_body.len, "the ball's only shape carries the ball's only contact");
    assert(from_shape[0].contact_id.index1 == from_body[0].contact_id.index1, "the same contact");
    assert(ball_shape.contact_capacity() >= 1, "and the shape reports capacity for it");
}

fn void a_contact_id_survives_a_step_and_dies_with_its_body() @test {
    b3::BodyId ball;
    b3::ShapeId ball_shape;
    b3::WorldId world = a_world_with_a_ball_resting_on_ground(&ball, &ball_shape);
    defer world.destroy();

    b3::ContactData[8] storage;
    b3::ContactData[] contacts = ball.contact_data(storage[..]);
    assert(contacts.len == 1, "the resting ball reports its one contact");

    b3::ContactId held = contacts[0].contact_id;

    world.step(1.0f / 60.0f, 4);
    assert(held.is_valid(), "a resting contact outlives a step");

    b3::ContactData refetched = held.data();
    assert(refetched.contact_manifolds().len >= 1, "and its geometry can be fetched again");

    ball.destroy();
    assert(!held.is_valid(), "destroying a body ends its contacts");
}
