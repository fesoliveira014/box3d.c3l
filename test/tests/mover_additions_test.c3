module b3_test;

import b3;

struct MoverSweep {
    bool hit;
    fault refusal;
    b3::BodyToiResult result;
    b3::ShapeId sphere;
}

// A half-metre sphere on a body ten metres along x, and a capsule mover of radius 0.25 starting
// three metres short of it and travelling six. The surfaces meet when the mover's axis is 0.75
// from the sphere's centre, 2.25 metres in: three eighths of the sweep.
fn MoverSweep the_sweep_of_a_mover_at_a_sphere(b3::Vec3 translation, b3::Vec3 body_shift) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::BodyDef body_def = b3::default_body_def();
    body_def.position = { 10.0, 0.0, 0.0 };
    b3::BodyId body = b3::create_body(world, &body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere ball = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId sphere = b3::create_sphere_shape(body, &shape_def, &ball)!!;

    b3::Capsule mover = { .center1 = { -3.0, -0.1, 0.0 }, .center2 = { -3.0, 0.1, 0.0 }, .radius = 0.25 };
    b3::WorldTransform start = body.transform();
    b3::WorldTransform end = start;
    end.p = (b3::Pos)(start.p + body_shift);

    b3::BodyToiResult? result = b3::body_time_of_impact_mover(body,
      origin: { 10.0, 0.0, 0.0 },
      mover: &mover,
      mover_translation: translation,
      filter: b3::default_query_filter(),
      body_transform1: start,
      body_transform2: end,);
    if (catch err = result) return { .hit = false, .refusal = err, .sphere = sphere };
    return { .hit = true, .result = result, .sphere = sphere };
}

fn void a_mover_swept_at_a_body_stops_where_the_surfaces_meet() @test {
    MoverSweep sweep = the_sweep_of_a_mover_at_a_sphere({ 6.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 });

    assert(sweep.hit);
    assert(sweep.result.fraction > 0.36 && sweep.result.fraction < 0.38,
        "three eighths of the sweep, short by box3d's closing slop at most");
    assert(sweep.result.point.x > 9.49 && sweep.result.point.x < 9.51,
        "the hit point is on the sphere's near side, in world space rather than relative to origin");
    assert(sweep.result.point.y > -0.01 && sweep.result.point.y < 0.01);
    assert(sweep.result.normal.x > -1.01 && sweep.result.normal.x < -0.99,
        "and the normal points from the body back towards the mover");
    assert(sweep.result.normal.y > -0.01 && sweep.result.normal.y < 0.01);
    assert(sweep.result.shape_id.index1 == sweep.sphere.index1
        && sweep.result.shape_id.generation == sweep.sphere.generation, "naming the sphere it struck");
}

fn void a_mover_sweep_follows_the_body_between_its_two_transforms() @test {
    MoverSweep sweep = the_sweep_of_a_mover_at_a_sphere({ 6.0, 0.0, 0.0 }, { -6.0, 0.0, 0.0 });

    assert(sweep.hit);
    assert(sweep.result.fraction > 0.175 && sweep.result.fraction < 0.19,
        "a sphere coming the other way as fast closes the gap in half the time: three sixteenths");
    assert(sweep.result.point.x > 8.36 && sweep.result.point.x < 8.39,
        "where the sphere had reached by then, not where it started");
}

fn void a_mover_sweep_that_passes_the_body_by_is_no_hit() @test {
    MoverSweep sweep = the_sweep_of_a_mover_at_a_sphere({ 0.0, 6.0, 0.0 }, { 0.0, 0.0, 0.0 });

    assert(!sweep.hit);
    assert(sweep.refusal == b3::NO_HIT);
}
