module b3_test;

import b3;

struct BodyExtents {
    float min_extent;
    b3::Vec3 max_extent;
    b3::Vec3 max_extent_origin;
}

fn BodyExtents the_extents_of(b3::BodyId body) {
    return {
        .min_extent = body.min_extent(),
        .max_extent = body.max_extent(),
        .max_extent_origin = body.max_extent_origin(),
    };
}

fn bool is_near(float value, float expected) => value > expected - 1.0e-4f && value < expected + 1.0e-4f;

// A capsule a metre off the body origin along x, standing along y: its centre of mass is its
// middle, so every axis of the two extents differs and a swap of any of them shows.
fn void a_dynamic_bodys_extents_are_measured_from_its_centre_of_mass() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = { 5.0, 3.0, -2.0 };
    b3::BodyId body = b3::create_body(world, &body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Capsule capsule = { .center1 = { 1.0, -0.5, 0.0 }, .center2 = { 1.0, 0.5, 0.0 }, .radius = 0.25 };
    b3::create_capsule_shape(body, &shape_def, &capsule)!!;

    BodyExtents read = the_extents_of(body);

    assert(is_near(read.min_extent, 0.25), "the capsule's radius is its thinnest");
    assert(is_near(read.max_extent.x, 0.25) && is_near(read.max_extent.y, 0.75)
        && is_near(read.max_extent.z, 0.25), "and it reaches its radius past its segment");
    assert(is_near(read.max_extent_origin.x, 1.25), "the origin's bound adds the metre to x");
    assert(is_near(read.max_extent_origin.y, 0.75) && is_near(read.max_extent_origin.z, 0.25),
        "and nothing to the axes the centre of mass is not offset along");
}

fn void a_static_bodys_extents_are_never_computed() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::BodyDef body_def = b3::default_body_def();
    b3::BodyId body = b3::create_body(world, &body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::create_sphere_shape(body, &shape_def, &sphere)!!;

    BodyExtents read = the_extents_of(body);

    assert(read.min_extent == 1.0e5, "exact: box3d's huge length at one unit per metre");
    assert(read.max_extent.x == 0.0 && read.max_extent.y == 0.0 && read.max_extent.z == 0.0);
    assert(read.max_extent_origin.x == 0.0 && read.max_extent_origin.y == 0.0
        && read.max_extent_origin.z == 0.0);
}

fn void the_safety_factor_starts_at_the_definitions_and_is_stored_as_given() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.safety_factor = 0.25;
    b3::BodyId body = b3::create_body(world, &body_def)!!;

    float from_the_definition = body.safety_factor();
    body.set_safety_factor(0.125);
    float set = body.safety_factor();

    assert(from_the_definition == 0.25, "exact: a power of two survives the float");
    assert(set == 0.125);
}

// A small sphere falling fast enough to cover twice its radius in a step: a safety factor past the
// motion lets it step straight through a thin slab, and the default one sweeps it to a stop.
fn float where_a_fast_sphere_comes_to_rest(float safety_factor) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = b3::create_body(world, &ground_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::BoxHull slab = b3::make_box_hull(5.0, 0.05, 5.0);
    b3::create_hull_shape(ground, &shape_def, &slab.base)!!;

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = { 0.0, 2.0, 0.0 };
    body_def.linear_velocity = { 0.0, -60.0, 0.0 };
    b3::BodyId body = b3::create_body(world, &body_def)!!;
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.2 };
    b3::create_sphere_shape(body, &shape_def, &sphere)!!;
    body.set_safety_factor(safety_factor);

    for (int i = 0; i < 20; i++) b3::world_step(world, 1.0f / 60.0f, 4);
    return body.position().y;
}

fn void a_safety_factor_past_the_motion_lets_a_fast_body_tunnel() @test {
    float swept = where_a_fast_sphere_comes_to_rest(0.5);
    float unswept = where_a_fast_sphere_comes_to_rest(100.0);

    assert(swept > 0.2 && swept < 0.3, "the default factor rests the sphere on the slab");
    assert(unswept < -1.0, "a factor no motion reaches lets it fall through");
}
