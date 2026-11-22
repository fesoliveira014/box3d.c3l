module b3_test;

import b3;
import std::math;
import std::atomic;

const float MATERIAL_STEP = 1.0f / 60.0f;

struct LaneMaterials {
    b3::ShapeId slippery_lane;
    b3::ShapeId grippy_lane;
    b3::ShapeId bouncy_floor;
}

struct ThreadCensus {
    int generation;
    int calls;
    int threads;
}

struct Conveyors {
    b3::ShapeId[2] belts;
    bool[2] running;
}

struct PileRun {
    ulong hash;
    int calls;
    int threads;
}

tlocal int census_generation_seen;

fn bool touches(b3::ShapeId a, b3::ShapeId b, b3::ShapeId wanted) {
    return same_shape(a, wanted) || same_shape(b, wanted);
}

fn void lane_materials(
  b3::ShapeId shape_a,
  b3::ShapeId shape_b,
  b3::ContactMaterial* material,
  void* context,
) {
    LaneMaterials* lanes = context;
    if (touches(shape_a, shape_b, lanes.slippery_lane)) material.friction = 0.1;
    if (touches(shape_a, shape_b, lanes.grippy_lane)) material.friction = 0.8;
    if (touches(shape_a, shape_b, lanes.bouncy_floor)) material.restitution = 0.8;
}

fn void unusable_materials(
  b3::ShapeId shape_a,
  b3::ShapeId shape_b,
  b3::ContactMaterial* material,
  void* context,
) {
    float infinity = (float)1.0e40;
    float not_a_number = infinity - infinity;
    material.friction = not_a_number;
    material.restitution = -1.0;
    material.rolling_resistance = infinity;
    material.tangent_velocity = {not_a_number, 0.0, 0.0};
}

fn void counting_materials(
  b3::ShapeId shape_a,
  b3::ShapeId shape_b,
  b3::ContactMaterial* material,
  void* context,
) {
    ThreadCensus* census = context;
    atomic::fetch_add(&census.calls, 1);
    if (census_generation_seen != census.generation) {
        census_generation_seen = census.generation;
        atomic::fetch_add(&census.threads, 1);
    }
    material.friction = (shape_a.index1 + shape_b.index1) % 2 == 0 ? 0.2 : 0.9;
}

fn void conveyor_belts(
  b3::ShapeId shape_a,
  b3::ShapeId shape_b,
  b3::ContactMaterial* material,
  void* context,
) {
    Conveyors* conveyors = context;
    foreach (i, belt : conveyors.belts) {
        if (!conveyors.running[i] || !touches(shape_a, shape_b, belt)) continue;
        material.tangent_velocity = {3.0, 0.0, 0.0};
    }
}

fn b3::ShapeId a_static_slab(b3::WorldId world, b3::Pos position, b3::Vec3 half_extents) {
    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = position;
    b3::BodyId slab = b3::create_body(world, &body_def)!!;
    b3::BoxHull box = b3::make_box_hull(half_extents.x, half_extents.y, half_extents.z);
    b3::ShapeDef shape_def = b3::default_shape_def();
    return b3::create_hull_shape(slab, &shape_def, &box.base)!!;
}

fn b3::BodyId a_sliding_box(b3::WorldId world, b3::Pos position) {
    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = position;
    b3::BodyId body = b3::create_body(world, &body_def)!!;
    b3::BoxHull box = b3::make_box_hull(0.25, 0.25, 0.25);
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    b3::create_hull_shape(body, &shape_def, &box.base)!!;
    return body;
}

<* The distances two boxes slide from 5 m/s on two lanes of default friction 0.6. *>
fn float[2] slide_distances(bool override, b3::ContactMaterialFn materials = &lane_materials) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    LaneMaterials lanes = {
        .slippery_lane = a_static_slab(world, {20.0, -0.5, 0.0}, {30.0, 0.5, 1.0}),
        .grippy_lane = a_static_slab(world, {20.0, -0.5, 5.0}, {30.0, 0.5, 1.0}),
        .bouncy_floor = {},
    };
    if (override) world.set_contact_material_callback(materials, &lanes);

    b3::BodyId[2] boxes = {
        a_sliding_box(world, {0.0, 0.25, 0.0}),
        a_sliding_box(world, {0.0, 0.25, 5.0}),
    };
    for (int i = 0; i < 30; i++) b3::world_step(world, MATERIAL_STEP, 4);
    float[2] start;
    foreach (i, box : boxes) {
        start[i] = box.position().x;
        box.set_linear_velocity({5.0, 0.0, 0.0});
    }
    for (int i = 0; i < 600; i++) b3::world_step(world, MATERIAL_STEP, 4);
    return { boxes[0].position().x - start[0], boxes[1].position().x - start[1] };
}

<* The highest point the ball's bottom reaches after its first bounce off the floor. *>
fn float bounce_height(
  bool override,
  b3::ContactMaterialFn materials = &lane_materials,
  float floor_restitution = 0.0,
) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::ShapeId floor = a_static_slab(world, {0.0, -0.5, 0.0}, {5.0, 0.5, 5.0});
    LaneMaterials lanes = { .bouncy_floor = floor };
    lanes.bouncy_floor.set_restitution(floor_restitution);
    if (override) world.set_contact_material_callback(materials, &lanes);

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 2.25, 0.0};
    b3::BodyId ball = b3::create_body(world, &body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.25};
    b3::create_sphere_shape(ball, &shape_def, &sphere)!!;

    bool bounced = false;
    float apex = 0.0;
    for (int i = 0; i < 240; i++) {
        b3::world_step(world, MATERIAL_STEP, 4);
        float vy = ball.linear_velocity().y;
        if (!bounced) {
            bounced = vy > 0.0 && ball.position().y < 0.5;
            continue;
        }
        apex = math::max(apex, ball.position().y);
        if (vy < 0.0) break;
    }
    return apex - 0.25;
}

fn PileRun a_pile_under_counting_materials(uint workers, int generation, bool recycling) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.worker_count = workers;
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    ThreadCensus census = { .generation = generation };
    world.set_contact_material_callback(&counting_materials, &census);
    a_static_slab(world, {0.0, -0.5, 0.0}, {20.0, 0.5, 20.0});

    b3::BodyId[256] bodies;
    b3::BoxHull box = b3::make_box_hull(0.3, 0.3, 0.3);
    foreach (i, &body : bodies) {
        b3::Pos position = {(float)(i % 16) - 7.5f, 0.3f + (float)(i / 64) * 0.7f, 0.0};
        position.z = (float)((i / 16) % 4) - 1.5f;
        b3::BodyDef body_def = b3::default_body_def();
        body_def.type = b3::BodyType.DYNAMIC;
        body_def.position = position;
        body_def.linear_velocity = {(float)(i % 3) - 1.0f, 0.0, (float)(i % 5) - 2.0f};
        body_def.enable_contact_recycling = recycling;
        *body = b3::create_body(world, &body_def)!!;
        b3::ShapeDef shape_def = b3::default_shape_def();
        shape_def.density = 1.0;
        b3::create_hull_shape(*body, &shape_def, &box.base)!!;
    }
    for (int i = 0; i < 120; i++) b3::world_step(world, MATERIAL_STEP, 4);

    return {
        .hash = hash_of_bodies(bodies[..]),
        .calls = atomic::fetch_add(&census.calls, 0),
        .threads = atomic::fetch_add(&census.threads, 0),
    };
}

fn void friction_set_per_contact_changes_how_far_a_box_slides() @test {
    float[2] mixed = slide_distances(override: false);
    float[2] overridden = slide_distances(override: true);
    float[2] refused = slide_distances(override: true, materials: &unusable_materials);

    // v^2 / (2 mu g) from 5 m/s under gravity 10: 2.08 m at the shapes' 0.6, 12.5 m at 0.1 and
    // 1.56 m at 0.8.
    assert(mixed[0] > 1.8 && mixed[0] < 2.4 && mixed[1] > 1.8 && mixed[1] < 2.4,
        "without the callback both lanes mix the shapes' friction of 0.6");
    assert(overridden[0] > 11.0 && overridden[0] < 14.0, "the slippery lane slides at 0.1");
    assert(overridden[1] > 1.3 && overridden[1] < 1.8, "the grippy lane holds at 0.8");
    assert(refused[0] == mixed[0] && refused[1] == mixed[1],
        "values the solver cannot use keep the mixed ones");
}

fn void restitution_set_per_contact_changes_the_bounce_height() @test {
    float dead = bounce_height(override: false);
    float lively = bounce_height(override: true);
    float mixed = bounce_height(override: false, floor_restitution: 0.5);
    float refused = bounce_height(
      override: true,
      materials: &unusable_materials,
      floor_restitution: 0.5,
    );

    // A 2 m drop at restitution e rises e^2 * 2 m: nothing at the shapes' 0, 1.28 m at 0.8 and
    // 0.5 m at 0.5.
    assert(dead < 0.05, "without the callback the ball does not bounce");
    assert(lively > 1.1 && lively < 1.45, "a restitution of 0.8 set on the contact bounces it");
    assert(mixed > 0.4 && mixed < 0.6, "a floor of restitution 0.5 bounces it half as high");
    assert(refused == mixed, "and a negative restitution written back keeps that bounce");
}

fn void the_material_callback_runs_on_the_step_workers_without_changing_the_result() @test {
    // Without recycling every touching contact updates on every step; with it, as by default,
    // only the contacts that moved do.
    foreach (recycling : (bool[2]){ false, true }) {
        int base = recycling ? 10 : 0;
        PileRun threaded = a_pile_under_counting_materials(4, base + 1, recycling);
        PileRun again = a_pile_under_counting_materials(4, base + 2, recycling);
        PileRun serial = a_pile_under_counting_materials(1, base + 3, recycling);

        assert(threaded.calls > (recycling ? 1000 : 10000), "the contacts asked for materials");
        assert(threaded.threads >= 2, "from more than one of the step's threads");
        assert(serial.threads == 1, "a single-threaded world calls it on the stepping thread only");
        assert(threaded.calls == serial.calls, "the worker count changes who calls, not how often");
        assert(threaded.hash == again.hash && threaded.hash == serial.hash,
            "and the pile lands bit for bit the same on four workers, twice, and on one");
    }
}

struct ConveyorRun {
    b3::WorldId world;
    Conveyors conveyors;
    b3::BodyId[2] crates;
}

<* Two crates resting on two belts, settled before anything is installed; sleep is off. *>
fn ConveyorRun* two_resting_crates(ConveyorRun* run) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.enable_sleep = false;
    run.world = b3::create_world(&world_def)!!;
    run.conveyors.belts = {
        a_static_slab(run.world, {0.0, -0.5, 0.0}, {20.0, 0.5, 1.0}),
        a_static_slab(run.world, {0.0, -0.5, 5.0}, {20.0, 0.5, 1.0}),
    };
    run.crates = {
        a_sliding_box(run.world, {0.0, 0.25, 0.0}),
        a_sliding_box(run.world, {0.0, 0.25, 5.0}),
    };
    for (int i = 0; i < 120; i++) b3::world_step(run.world, MATERIAL_STEP, 4);
    return run;
}

fn float[2] crate_speeds_over_a_second(ConveyorRun* run) {
    float[2] fastest;
    for (int i = 0; i < 60; i++) {
        b3::world_step(run.world, MATERIAL_STEP, 4);
        foreach (j, crate : run.crates) {
            fastest[j] = math::max(fastest[j], crate.linear_velocity().length());
        }
    }
    return fastest;
}

fn void a_callback_set_after_the_bodies_settle_reaches_their_resting_contacts() @test {
    ConveyorRun run;
    two_resting_crates(&run);
    defer b3::destroy_world(run.world);

    run.conveyors.running = { true, true };
    run.world.set_contact_material_callback(&conveyor_belts, &run.conveyors);
    float[2] speeds = crate_speeds_over_a_second(&run);
    assert(speeds[0] > 2.5 && speeds[0] < 3.5 && speeds[1] > 2.5 && speeds[1] < 3.5,
        "both belts carry their resting crates at 3 m/s");
}

fn void changed_answers_reach_resting_contacts_once_invalidated() @test {
    ConveyorRun run;
    two_resting_crates(&run);
    defer b3::destroy_world(run.world);
    run.world.set_contact_material_callback(&conveyor_belts, &run.conveyors);
    crate_speeds_over_a_second(&run);

    run.conveyors.running = { true, true };
    float[2] unannounced = crate_speeds_over_a_second(&run);
    assert(unannounced[0] < 0.01 && unannounced[1] < 0.01,
        "a resting contact is recycled, so it keeps the answer it had until it is invalidated");

    b3::shape_invalidate_contact_materials(run.conveyors.belts[1]);
    float[2] one_belt = crate_speeds_over_a_second(&run);
    assert(one_belt[0] < 0.01, "invalidating one belt leaves the other's contact as it was");
    assert(one_belt[1] > 2.5 && one_belt[1] < 3.5, "and starts the crate on the belt invalidated");

    b3::world_invalidate_contact_materials(run.world);
    float[2] both = crate_speeds_over_a_second(&run);
    assert(both[0] > 2.5 && both[0] < 3.5, "invalidating the world starts the other crate as well");
}
