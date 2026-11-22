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

fn bool touches(b3::ShapeId a, b3::ShapeId b, b3::ShapeId wanted) {
    return same_shape(a, wanted) || same_shape(b, wanted);
}

fn void lane_materials(b3::ShapeId shape_a, b3::ShapeId shape_b, b3::ContactMaterial* material, void* context) {
    LaneMaterials* lanes = context;
    if (touches(shape_a, shape_b, lanes.slippery_lane)) material.friction = 0.1;
    if (touches(shape_a, shape_b, lanes.grippy_lane)) material.friction = 0.8;
    if (touches(shape_a, shape_b, lanes.bouncy_floor)) material.restitution = 0.8;
}

fn void unusable_materials(b3::ShapeId shape_a, b3::ShapeId shape_b, b3::ContactMaterial* material, void* context) {
    float infinity = (float)1.0e40;
    float not_a_number = infinity - infinity;
    material.friction = not_a_number;
    material.restitution = -1.0;
    material.rolling_resistance = infinity;
    material.tangent_velocity = {not_a_number, 0.0, 0.0};
}

fn b3::ShapeId a_static_slab(b3::WorldId world, b3::Pos position, float half_x, float half_z) {
    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = position;
    b3::BodyId slab = b3::create_body(world, &body_def)!!;
    b3::BoxHull box = b3::make_box_hull(half_x, 0.5, half_z);
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
        .slippery_lane = a_static_slab(world, {20.0, -0.5, 0.0}, 30.0, 1.0),
        .grippy_lane = a_static_slab(world, {20.0, -0.5, 5.0}, 30.0, 1.0),
        .bouncy_floor = {},
    };
    if (override) world.set_contact_material_callback(materials, &lanes);

    b3::BodyId[2] boxes = { a_sliding_box(world, {0.0, 0.25, 0.0}), a_sliding_box(world, {0.0, 0.25, 5.0}) };
    for (int i = 0; i < 30; i++) b3::world_step(world, MATERIAL_STEP, 4);
    float[2] start;
    foreach (i, box : boxes) {
        start[i] = box.position().x;
        box.set_linear_velocity({5.0, 0.0, 0.0});
    }
    for (int i = 0; i < 600; i++) b3::world_step(world, MATERIAL_STEP, 4);
    return { boxes[0].position().x - start[0], boxes[1].position().x - start[1] };
}

fn void friction_set_per_contact_changes_how_far_a_box_slides() @test {
    float[2] mixed = slide_distances(override: false);
    float[2] overridden = slide_distances(override: true);

    // v^2 / (2 mu g) from 5 m/s under gravity 10: 2.08 m at the shapes' 0.6, 12.5 m at 0.1 and
    // 1.56 m at 0.8.
    assert(mixed[0] > 1.8 && mixed[0] < 2.4 && mixed[1] > 1.8 && mixed[1] < 2.4,
        "without the callback both lanes mix the shapes' friction of 0.6");
    assert(overridden[0] > 11.0 && overridden[0] < 14.0, "the contact on the slippery lane slides at 0.1");
    assert(overridden[1] > 1.3 && overridden[1] < 1.8, "the contact on the grippy lane holds at 0.8");
    float[2] refused = slide_distances(override: true, materials: &unusable_materials);
    assert(refused[0] == mixed[0] && refused[1] == mixed[1], "values the solver cannot use keep the mixed ones");
}

<* The highest point the ball's centre reaches after its first bounce off the floor. *>
fn float bounce_height(bool override, b3::ContactMaterialFn materials = &lane_materials, float floor_restitution = 0.0) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    LaneMaterials lanes = { .bouncy_floor = a_static_slab(world, {0.0, -0.5, 0.0}, 5.0, 5.0) };
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

fn void restitution_set_per_contact_changes_the_bounce_height() @test {
    float dead = bounce_height(override: false);
    float lively = bounce_height(override: true);

    // A 2 m drop at restitution e rises e^2 * 2 m: nothing at the shapes' 0, 1.28 m at 0.8.
    assert(dead < 0.05, "without the callback the ball does not bounce");
    assert(lively > 1.1 && lively < 1.45, "a restitution of 0.8 set on the contact bounces it 1.28 m");
    float mixed = bounce_height(override: false, floor_restitution: 0.5);
    float refused = bounce_height(override: true, materials: &unusable_materials, floor_restitution: 0.5);
    assert(mixed > 0.4 && mixed < 0.6, "a floor of restitution 0.5 bounces the ball 0.5 m");
    assert(refused == mixed, "and a negative restitution written back keeps that bounce");
}

struct ThreadCensus {
    int generation;
    int calls;
    int threads;
}

tlocal int census_generation_seen;

fn void counting_materials(b3::ShapeId shape_a, b3::ShapeId shape_b, b3::ContactMaterial* material, void* context) {
    ThreadCensus* census = context;
    atomic::fetch_add(&census.calls, 1);
    if (census_generation_seen != census.generation) {
        census_generation_seen = census.generation;
        atomic::fetch_add(&census.threads, 1);
    }
    material.friction = (shape_a.index1 + shape_b.index1) % 2 == 0 ? 0.2 : 0.9;
}

struct PileRun {
    ulong hash;
    int calls;
    int threads;
}

fn PileRun a_pile_under_counting_materials(uint workers, int generation) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.worker_count = workers;
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    ThreadCensus census = { .generation = generation };
    world.set_contact_material_callback(&counting_materials, &census);
    a_static_slab(world, {0.0, -0.5, 0.0}, 20.0, 20.0);

    b3::BodyId[256] bodies;
    b3::BoxHull box = b3::make_box_hull(0.3, 0.3, 0.3);
    foreach (i, &body : bodies) {
        b3::BodyDef body_def = b3::default_body_def();
        body_def.type = b3::BodyType.DYNAMIC;
        body_def.position = {(float)(i % 16) - 7.5f, 0.3f + (float)(i / 64) * 0.7f, (float)((i / 16) % 4) - 1.5f};
        body_def.linear_velocity = {(float)(i % 3) - 1.0f, 0.0, (float)(i % 5) - 2.0f};
        body_def.enable_contact_recycling = false;
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

fn void the_material_callback_runs_on_the_step_workers_without_changing_the_result() @test {
    PileRun threaded = a_pile_under_counting_materials(workers: 4, generation: 1);
    PileRun again = a_pile_under_counting_materials(workers: 4, generation: 2);
    PileRun serial = a_pile_under_counting_materials(workers: 1, generation: 3);

    assert(threaded.calls > 10000, "every touching contact of every step asked for its material");
    assert(threaded.threads >= 2, "and the calls came from more than one of the step's threads");
    assert(serial.threads == 1, "a single-threaded world calls it on the stepping thread alone");
    assert(threaded.calls == serial.calls, "the worker count changes who calls it, not how often");
    assert(threaded.hash == again.hash && threaded.hash == serial.hash,
        "and the pile lands bit for bit the same on four workers, twice, and on one");
}
