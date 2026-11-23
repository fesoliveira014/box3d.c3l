module b3_test;

import b3;
import std::math;

const int STACK_HEIGHT = 6;
const float STEP = 1.0f / 60.0f;
// Ten kilometres out: float spacing there is a millimetre, so a shift that rounded the stack apart
// would show up as contact impulses and a woken island.
const b3::Pos FAR_BASE = {10000.0, 0.0, -10000.0};

struct Stack {
    b3::WorldId world;
    b3::BodyId ground;
    b3::ShapeId ground_shape;
    b3::BodyId[STACK_HEIGHT] boxes;
    b3::ShapeId[STACK_HEIGHT] shapes;
}

struct Pendulum {
    b3::WorldId world;
    b3::BodyId anchor;
    b3::BodyId[3] links;
    b3::SphericalJointId[3] joints;
}

fn Stack a_box_stack_at(b3::Pos base, bool enable_sleep) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.enable_sleep = enable_sleep;
    Stack stack = { .world = b3::create_world(&world_def)!! };

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = (b3::Pos)(base + (b3::Vec3){0.0, -0.5, 0.0});
    stack.ground = b3::create_body(stack.world, &body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(20.0, 0.5, 20.0);
    b3::ShapeDef shape_def = b3::default_shape_def();
    stack.ground_shape = b3::create_hull_shape(stack.ground, &shape_def, &ground_box.base)!!;

    b3::BoxHull box = b3::make_box_hull(0.5, 0.5, 0.5);
    for (int i = 0; i < STACK_HEIGHT; i++) {
        body_def = b3::default_body_def();
        body_def.type = b3::BodyType.DYNAMIC;
        body_def.position = (b3::Pos)(base + (b3::Vec3){0.0, 0.5 + (float)i, 0.0});
        stack.boxes[i] = b3::create_body(stack.world, &body_def)!!;
        shape_def = b3::default_shape_def();
        shape_def.density = 1.0;
        stack.shapes[i] = b3::create_hull_shape(stack.boxes[i], &shape_def, &box.base)!!;
    }
    return stack;
}

fn void settle_asleep(Stack* stack) {
    for (int i = 0; i < 600 && stack.world.awake_body_count() > 0; i++) {
        b3::world_step(stack.world, STEP, 4);
    }
}

fn bool same_shape(b3::ShapeId a, b3::ShapeId b) {
    return a.index1 == b.index1 && a.world0 == b.world0 && a.generation == b.generation;
}

<* How far `after` is from `before` moved by -origin, at the worse of its two corners. *>
fn float box_moved_by(b3::Aabb after, b3::Aabb before, b3::Pos origin) {
    float lower = (after.lower_bound - (before.lower_bound - origin)).length();
    float upper = (after.upper_bound - (before.upper_bound - origin)).length();
    return math::max(lower, upper);
}

fn float fastest_box(Stack* stack) {
    float fastest = 0.0;
    foreach (box : stack.boxes) {
        fastest = math::max(fastest, box.linear_velocity().length());
        fastest = math::max(fastest, box.angular_velocity().length());
    }
    return fastest;
}

fn float fastest_box_over_steps(Stack* stack, int steps) {
    float fastest = 0.0;
    for (int i = 0; i < steps; i++) {
        b3::world_step(stack.world, STEP, 4);
        fastest = math::max(fastest, fastest_box(stack));
    }
    return fastest;
}

fn float largest_offset_from(Stack* stack, b3::Pos[STACK_HEIGHT]* expected) {
    float largest = 0.0;
    foreach (i, box : stack.boxes) {
        largest = math::max(largest, (box.position() - (*expected)[i]).length());
    }
    return largest;
}

fn void positions_shifted(Stack* stack, b3::Pos origin, b3::Pos[STACK_HEIGHT]* out) {
    foreach (i, box : stack.boxes) (*out)[i] = (b3::Pos)(box.position() - origin);
}

fn b3::RayResult closest_hit(b3::WorldId world, b3::Vec3 origin, b3::Vec3 translation) {
    return b3::world_cast_ray_closest(
      world: world,
      origin: (b3::Pos)origin,
      translation: translation,
      filter: b3::default_query_filter(),
    )!!;
}

fn b3::ShapeId[] overlapping(b3::WorldId world, b3::Aabb region, b3::ShapeId[] buffer) {
    return b3::world_overlap_aabb_all(
      world: world,
      aabb: region,
      filter: b3::default_query_filter(),
      buffer: buffer,
    );
}

fn void a_resting_stack_far_out_stays_at_rest_when_the_origin_moves_onto_it() @test {
    Stack stack = a_box_stack_at(FAR_BASE, enable_sleep: false);
    defer b3::destroy_world(stack.world);
    fastest_box_over_steps(&stack, 300);
    float settled = fastest_box_over_steps(&stack, 60);

    b3::Pos[STACK_HEIGHT] expected;
    positions_shifted(&stack, FAR_BASE, &expected);
    b3::world_shift_origin(stack.world, FAR_BASE);
    float moved_by_shift = largest_offset_from(&stack, &expected);

    float after = fastest_box_over_steps(&stack, 120);
    float drift = largest_offset_from(&stack, &expected);
    assert(moved_by_shift == 0.0, "the shift onto the stack is exact: position - origin");
    assert(settled < 1e-4, "the stack had settled before the shift");
    assert(after < 1e-4, "no impulse follows: every box stays under 0.1 mm/s for the next 2 s");
    assert(drift < 1e-4, "nor creep: the boxes stay within 0.1 mm of where they rested");
}

fn void a_resting_stack_stays_at_rest_when_the_origin_moves_by_an_inexact_offset() @test {
    Stack stack = a_box_stack_at({0.0, 0.0, 0.0}, enable_sleep: false);
    defer b3::destroy_world(stack.world);
    fastest_box_over_steps(&stack, 300);
    float settled = fastest_box_over_steps(&stack, 60);

    b3::Pos origin = {-1234.567, 3.25, 4321.123};
    b3::Pos[STACK_HEIGHT] expected;
    positions_shifted(&stack, origin, &expected);
    b3::world_shift_origin(stack.world, origin);
    float moved_by_shift = largest_offset_from(&stack, &expected);

    float after = fastest_box_over_steps(&stack, 120);
    float drift = largest_offset_from(&stack, &expected);
    assert(moved_by_shift < 1e-3, "each box lands within float rounding of position - origin");
    assert(settled < 1e-4, "the stack had settled before the shift");
    assert(after < 1e-4, "no impulse follows: every box stays under 0.1 mm/s for the next 2 s");
    assert(drift < 1e-4, "nor creep: the boxes stay within 0.1 mm of where they rested");
}

fn void a_sleeping_stack_stays_asleep_across_a_shift() @test {
    Stack stack = a_box_stack_at(FAR_BASE, enable_sleep: true);
    defer b3::destroy_world(stack.world);
    settle_asleep(&stack);
    assert(stack.world.awake_body_count() == 0, "the stack fell asleep before the shift");

    b3::Pos[STACK_HEIGHT] expected;
    positions_shifted(&stack, FAR_BASE, &expected);
    b3::Aabb ground_box = stack.ground_shape.aabb();
    b3::Aabb top_box = stack.shapes[STACK_HEIGHT - 1].aabb();
    b3::world_shift_origin(stack.world, FAR_BASE);
    assert(box_moved_by(stack.ground_shape.aabb(), ground_box, FAR_BASE) < 2e-3,
        "the static bounds moved");
    assert(box_moved_by(stack.shapes[STACK_HEIGHT - 1].aabb(), top_box, FAR_BASE) < 2e-3,
        "and so did the bounds of a sleeping box, which no step recomputes");
    for (int i = 0; i < 60; i++) b3::world_step(stack.world, STEP, 4);

    assert(stack.world.awake_body_count() == 0, "the shift wakes nothing, nor the next second");
    foreach (box : stack.boxes) assert(!box.is_awake());
    assert(largest_offset_from(&stack, &expected) == 0.0,
        "the sleeping boxes sit exactly at position - origin");
    b3::Pos ground = stack.ground.position();
    assert(ground.x == 0.0 && ground.y == -0.5 && ground.z == 0.0, "the static ground moved too");
}

fn void queries_after_a_shift_return_the_same_hits_shifted() @test {
    Stack stack = a_box_stack_at(FAR_BASE, enable_sleep: true);
    defer b3::destroy_world(stack.world);
    settle_asleep(&stack);

    b3::Vec3 down = {0.0, -20.0, 0.0};
    b3::Vec3 sideways = {-40.0, 0.0, 0.0};
    b3::Vec3 above = {0.1, 10.0, 0.1};
    b3::Vec3 beside = {20.0, 2.6, 0.2};
    b3::Aabb region = {.lower_bound = {-0.4, 1.2, -0.4}, .upper_bound = {0.4, 3.8, 0.4}};
    b3::Aabb far_region = {
        .lower_bound = region.lower_bound + FAR_BASE,
        .upper_bound = region.upper_bound + FAR_BASE,
    };

    b3::RayResult top_before = closest_hit(stack.world, FAR_BASE + above, down);
    b3::RayResult side_before = closest_hit(stack.world, FAR_BASE + beside, sideways);
    b3::ShapeId[16] buffer_before;
    b3::ShapeId[] overlap_before = overlapping(stack.world, far_region, buffer_before[..]);

    b3::world_shift_origin(stack.world, FAR_BASE);

    b3::RayResult top_after = closest_hit(stack.world, above, down);
    b3::RayResult side_after = closest_hit(stack.world, beside, sideways);
    b3::ShapeId[16] buffer_after;
    b3::ShapeId[] overlap_after = overlapping(stack.world, region, buffer_after[..]);

    assert(same_shape(top_before.shape_id, stack.shapes[STACK_HEIGHT - 1]));
    assert(same_shape(top_after.shape_id, top_before.shape_id), "the ray down hits the top box");
    assert(same_shape(side_before.shape_id, stack.shapes[2]));
    assert(same_shape(side_after.shape_id, side_before.shape_id), "the ray across hits the third");
    assert((top_after.point - (top_before.point - FAR_BASE)).length() < 1e-3);
    assert((side_after.point - (side_before.point - FAR_BASE)).length() < 1e-3);
    assert(math::abs(top_after.fraction - top_before.fraction) < 1e-4);
    assert(math::abs(side_after.fraction - side_before.fraction) < 1e-4);
    assert(overlap_before.len == 3, "the region spans the second, third and fourth box");
    assert(overlap_after.len == overlap_before.len);
    foreach (shape : overlap_before) {
        bool found = false;
        foreach (other : overlap_after) found = found || same_shape(other, shape);
        assert(found, "the overlap finds the same shapes after the shift");
    }
}

fn Pendulum a_swinging_chain_at(b3::Pos base) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.enable_sleep = false;
    Pendulum chain = { .world = b3::create_world(&world_def)!! };

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = (b3::Pos)(base + (b3::Vec3){0.0, 10.0, 0.0});
    chain.anchor = b3::create_body(chain.world, &body_def)!!;

    b3::BodyId previous = chain.anchor;
    b3::Capsule capsule = {.center1 = {0.0, 0.0, 0.0}, .center2 = {1.0, 0.0, 0.0}, .radius = 0.1};
    for (int i = 0; i < 3; i++) {
        body_def = b3::default_body_def();
        body_def.type = b3::BodyType.DYNAMIC;
        body_def.position = (b3::Pos)(base + (b3::Vec3){(float)i, 10.0, 0.0});
        chain.links[i] = b3::create_body(chain.world, &body_def)!!;
        b3::ShapeDef shape_def = b3::default_shape_def();
        shape_def.density = 1.0;
        b3::create_capsule_shape(chain.links[i], &shape_def, &capsule)!!;

        b3::SphericalJointDef joint_def = b3::default_spherical_joint_def();
        joint_def.body_id_a = previous;
        joint_def.body_id_b = chain.links[i];
        joint_def.local_frame_a.q = {0.0, 0.0, 0.0, 1.0};
        joint_def.local_frame_b.q = {0.0, 0.0, 0.0, 1.0};
        joint_def.local_frame_a.p = {i == 0 ? 0.0 : 1.0, 0.0, 0.0};
        chain.joints[i] = b3::create_spherical_joint(chain.world, &joint_def)!!;
        previous = chain.links[i];
    }
    return chain;
}

fn b3::Pos anchor_on_a(b3::JointId joint) => joint.body_a().world_point(joint.local_frame_a().p);
fn b3::Pos anchor_on_b(b3::JointId joint) => joint.body_b().world_point(joint.local_frame_b().p);

fn void joints_keep_their_anchors_across_a_shift() @test {
    Pendulum shifted = a_swinging_chain_at(FAR_BASE);
    defer b3::destroy_world(shifted.world);
    Pendulum reference = a_swinging_chain_at(FAR_BASE);
    defer b3::destroy_world(reference.world);
    for (int i = 0; i < 30; i++) {
        b3::world_step(shifted.world, STEP, 4);
        b3::world_step(reference.world, STEP, 4);
    }

    b3::Transform[3] frames_before;
    b3::Pos[3] anchors_before;
    foreach (i, joint : shifted.joints) {
        frames_before[i] = joint.local_frame_a();
        anchors_before[i] = anchor_on_a(joint);
    }
    b3::world_shift_origin(shifted.world, FAR_BASE);

    float worst_anchor = 0.0;
    foreach (i, joint : shifted.joints) {
        assert(joint.local_frame_a().p == frames_before[i].p, "the frames are body-local and stay");
        b3::Pos expected = (b3::Pos)(anchors_before[i] - FAR_BASE);
        worst_anchor = math::max(worst_anchor, (anchor_on_a(joint) - expected).length());
    }

    float worst_gap = 0.0;
    float worst_track = 0.0;
    for (int i = 0; i < 120; i++) {
        b3::world_step(shifted.world, STEP, 4);
        b3::world_step(reference.world, STEP, 4);
        foreach (j, joint : shifted.joints) {
            worst_gap = math::max(worst_gap, (anchor_on_a(joint) - anchor_on_b(joint)).length());
            b3::Pos expected = (b3::Pos)(reference.links[j].position() - FAR_BASE);
            worst_track = math::max(worst_track, (shifted.links[j].position() - expected).length());
        }
    }
    assert(worst_anchor < 1e-3, "each anchor lands on its old world point minus the origin");
    assert(worst_gap < 0.01, "the joints hold their anchors together while the chain swings on");
    assert(worst_track < 0.01, "and the shifted chain swings within 1 cm of an unshifted twin");
}

fn ulong hash_of_bodies(b3::BodyId[] bodies) {
    ulong hash = 14695981039346656037UL;
    foreach (body : bodies) {
        b3::WorldTransform pose = body.transform();
        float[7] words = { pose.p.x, pose.p.y, pose.p.z, pose.q.i, pose.q.j, pose.q.k, pose.q.l };
        foreach (word : words) hash = (hash ^ bitcast(word, uint)) * 1099511628211UL;
    }
    return hash;
}

fn ulong a_tumbling_pile_hash(
  bool shift,
  uint workers,
  record::Recording* recording = null,
  bool invalidate = false,
) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.worker_count = workers;
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = (b3::Pos)(FAR_BASE + (b3::Vec3){0.0, -0.5, 0.0});
    b3::BodyId ground = b3::create_body(world, &body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(20.0, 0.5, 20.0);
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::ShapeId ground_shape = b3::create_hull_shape(ground, &shape_def, &ground_box.base)!!;

    b3::BodyId[40] bodies;
    b3::BoxHull box = b3::make_box_hull(0.4, 0.3, 0.5);
    foreach (i, &body : bodies) {
        b3::Vec3 offset = {(float)(i % 4) * 0.7f, 1.0f + (float)i * 0.8f, (float)(i % 3) * 0.3f};
        body_def = b3::default_body_def();
        body_def.type = b3::BodyType.DYNAMIC;
        body_def.position = (b3::Pos)(FAR_BASE + offset);
        body_def.angular_velocity = {(float)(i % 5), 1.0, (float)(i % 2)};
        *body = b3::create_body(world, &body_def)!!;
        shape_def = b3::default_shape_def();
        shape_def.density = 1.0;
        b3::create_hull_shape(*body, &shape_def, &box.base)!!;
    }

    if (recording) record::world_start_recording(world, recording)!!;
    for (int i = 0; i < 180; i++) {
        if (shift && i == 60) b3::world_shift_origin(world, FAR_BASE);
        if (invalidate && i == 120) b3::world_invalidate_contact_materials(world);
        if (invalidate && i == 150) b3::shape_invalidate_contact_materials(ground_shape);
        b3::world_step(world, STEP, 4);
    }
    if (recording) record::world_stop_recording(world);
    return hash_of_bodies(bodies[..]);
}

fn void a_shifted_run_is_deterministic() @test {
    ulong shifted = a_tumbling_pile_hash(shift: true, workers: 4);
    ulong plain = a_tumbling_pile_hash(shift: false, workers: 4);
    assert(a_tumbling_pile_hash(shift: true, workers: 4) == shifted, "two shifted runs, one hash");
    assert(a_tumbling_pile_hash(shift: false, workers: 4) == plain, "two unshifted runs, one hash");
    assert(a_tumbling_pile_hash(shift: true, workers: 1) == shifted, "on any number of workers");
    assert(shifted != plain, "and the hash sees the shift, so the checks above test something");
}

fn void a_recorded_shift_and_invalidation_replay_to_the_states_recorded() @test {
    record::Recording* recording = record::create_recording(0);
    defer record::destroy_recording(recording);
    a_tumbling_pile_hash(shift: true, workers: 1, recording: recording, invalidate: true);
    assert(record::validate_replay(recording.data(), 1),
        "the replay shifts and invalidates contacts where the session did");
}

fn void a_fast_body_keeps_its_continuous_sweep_across_a_shift() @test {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.enable_sleep = false;
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = (b3::Pos)(FAR_BASE + (b3::Vec3){5.0, 1.0, 0.0});
    b3::BodyId wall = b3::create_body(world, &body_def)!!;
    b3::BoxHull slab = b3::make_box_hull(0.05, 2.0, 2.0);
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::create_hull_shape(wall, &shape_def, &slab.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = (b3::Pos)(FAR_BASE + (b3::Vec3){0.0, 1.0, 0.0});
    body_def.gravity_scale = 0.0;
    body_def.linear_velocity = {200.0, 0.0, 0.0};
    b3::BodyId ball = b3::create_body(world, &body_def)!!;
    shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    shape_def.enable_hit_events = true;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.1};
    b3::create_sphere_shape(ball, &shape_def, &sphere)!!;

    // One step leaves the ball 1.7 m short of the wall at 3.3 m per step, so the next step's sweep
    // starts at the shifted previous centre or the ball passes through.
    b3::world_step(world, STEP, 4);
    b3::world_shift_origin(world, FAR_BASE);
    b3::world_step(world, STEP, 4);
    float x = ball.position().x;
    assert(x < 4.95 && x > 4.5, "the sweep stopped the ball at the wall's near face");

    b3::ContactHitEvent[] hits = world.contact_hit_events();
    for (int i = 0; i < 10 && hits.len == 0; i++) {
        b3::world_step(world, STEP, 4);
        hits = world.contact_hit_events();
    }
    b3::BodyMoveEvent[] moves = world.body_move_events();
    assert(hits.len == 1 && moves.len == 1, "the ball reports its hit on the wall and its move");
    b3::Pos hit_point = hits[0].point;
    b3::Pos moved_to = moves[0].transform.p;
    assert(hit_point.x > 4.8 && hit_point.x < 5.0 && math::abs(hit_point.y - 1.0) < 0.01,
        "the hit is at the wall's near face in the shifted frame");

    b3::Pos again = {100.0, -2.0, 30.0};
    b3::world_shift_origin(world, again);
    hits = world.contact_hit_events();
    moves = world.body_move_events();
    assert((hits[0].point - (hit_point - again)).length() < 1e-5, "a shift moves the hit points");
    assert((moves[0].transform.p - (moved_to - again)).length() < 1e-5, "and the last move events");
}

fn void a_box_resting_on_a_mesh_stays_at_rest_across_a_shift() @test {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.enable_sleep = false;
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);
    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 16,
      z_count: 16,
      cell_width: 0.25,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer b3::destroy_mesh(grid);

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = FAR_BASE;
    b3::BodyId ground = b3::create_body(world, &body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::create_mesh_shape(ground, &shape_def, grid, {1.0, 1.0, 1.0})!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = (b3::Pos)(FAR_BASE + (b3::Vec3){0.3, 0.5, 0.2});
    b3::BodyId box = b3::create_body(world, &body_def)!!;
    shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    b3::BoxHull cube = b3::make_box_hull(0.5, 0.5, 0.5);
    b3::create_hull_shape(box, &shape_def, &cube.base)!!;
    for (int i = 0; i < 300; i++) b3::world_step(world, STEP, 4);

    b3::Pos expected = (b3::Pos)(box.position() - FAR_BASE);
    b3::world_shift_origin(world, FAR_BASE);
    float fastest = 0.0;
    for (int i = 0; i < 120; i++) {
        b3::world_step(world, STEP, 4);
        fastest = math::max(fastest, box.linear_velocity().length());
    }
    assert(fastest < 1e-4, "the mesh contact carries over without an impulse");
    assert((box.position() - expected).length() < 1e-4, "and the box stays where it rested");
}

const b3::Pos ODD_BASE = {3000.0, 0.0, -3000.0};

<* The largest change of any box's position relative to the bottom box, against `reference`. *>
fn float stack_distortion(Stack* stack, b3::Vec3[STACK_HEIGHT]* reference) {
    float largest = 0.0;
    foreach (i, box : stack.boxes) {
        b3::Vec3 relative = box.position() - stack.boxes[0].position();
        largest = math::max(largest, (relative - (*reference)[i]).length());
    }
    return largest;
}

fn void stack_shape(Stack* stack, b3::Vec3[STACK_HEIGHT]* out) {
    foreach (i, box : stack.boxes) (*out)[i] = box.position() - stack.boxes[0].position();
}

fn void inexact_shifts_round_a_stack_by_a_float_step_and_never_more() @test {
    // The three offsets add up to zero, so the stack wanders within a few kilometres of its start.
    b3::Pos[3] offsets;
    offsets[0] = {1234.567, 2345.6789, -987.65};
    offsets[1] = {-321.123, 77.777, 4444.4};
    offsets[2] = (b3::Pos)(-(offsets[0] + offsets[1]));

    // A large inexact vertical offset rounds every gap in the stack.
    Stack single = a_box_stack_at(ODD_BASE, enable_sleep: false);
    defer b3::destroy_world(single.world);
    fastest_box_over_steps(&single, 300);
    b3::world_shift_origin(single.world, {0.0, 2345.6789, 0.0});
    float single_after = fastest_box_over_steps(&single, 120);

    // Asleep: back and forth a thousand times, then two thousand different offsets.
    Stack sleeping = a_box_stack_at(ODD_BASE, enable_sleep: true);
    defer b3::destroy_world(sleeping.world);
    settle_asleep(&sleeping);
    b3::Vec3[STACK_HEIGHT] before;
    stack_shape(&sleeping, &before);
    for (int i = 0; i < 1000; i++) {
        b3::world_shift_origin(sleeping.world, offsets[0]);
        b3::world_shift_origin(sleeping.world, (b3::Pos)(-offsets[0]));
    }
    float back_and_forth = stack_distortion(&sleeping, &before);
    bool asleep_after_back_and_forth = sleeping.world.awake_body_count() == 0;
    for (int i = 0; i < 2000; i++) b3::world_shift_origin(sleeping.world, offsets[i % 3]);
    float wandering = stack_distortion(&sleeping, &before);
    bool asleep_after_wandering = sleeping.world.awake_body_count() == 0;
    sleeping.boxes[STACK_HEIGHT - 1].set_awake(true);
    float woken = fastest_box_over_steps(&sleeping, 120);

    // Awake: a shift before every step for a thousand steps.
    Stack awake = a_box_stack_at(ODD_BASE, enable_sleep: false);
    defer b3::destroy_world(awake.world);
    fastest_box_over_steps(&awake, 300);
    float every_step = 0.0;
    for (int i = 0; i < 1000; i++) {
        b3::Pos offset = i % 2 == 0 ? offsets[i % 3] : (b3::Pos)(-offsets[(i - 1) % 3]);
        b3::world_shift_origin(awake.world, offset);
        every_step = math::max(every_step, fastest_box_over_steps(&awake, 1));
    }

    // Measured 0.94 mm/s, 0.12 mm, 0.27 mm, 2.3 mm/s and 1.0 mm/s: a float step at 5 km is 0.5 mm,
    // far below the sleep threshold of 5 cm/s.
    assert(single_after < 3e-3, "one inexact vertical shift leaves the stack under 3 mm/s");
    assert(asleep_after_back_and_forth && asleep_after_wandering, "no shift wakes the stack");
    assert(back_and_forth < 5e-4, "a thousand shifts there and back distort it by under 0.5 mm");
    assert(wandering < 1e-3, "two thousand different shifts by under 1 mm: no build-up");
    assert(woken < 0.01, "woken afterwards, it settles under 1 cm/s");
    assert(every_step < 5e-3, "and shifted before each of 1000 steps it stays under 5 mm/s");
}

struct WallTwin {
    b3::WorldId world;
    b3::BodyId crate;
}

<* A crate resting on a floor 0.10 m short of a mesh wall facing it. *>
fn WallTwin a_crate_before_a_mesh_wall(b3::MeshData* grid) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.enable_sleep = false;
    WallTwin twin = { .world = b3::create_world(&world_def)!! };

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId floor = b3::create_body(twin.world, &body_def)!!;
    b3::BoxHull slab = b3::make_box_hull(5.0, 0.5, 5.0);
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::create_hull_shape(floor, &shape_def, &slab.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.6, 1.0, 0.0};
    // A quarter turn about z stands the grid up with its face toward -x, toward the crate.
    body_def.rotation = {0.0, 0.0, 0.70710678, 0.70710678};
    b3::BodyId wall = b3::create_body(twin.world, &body_def)!!;
    b3::create_mesh_shape(wall, &shape_def, grid, {1.0, 1.0, 1.0})!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 0.5, 0.0};
    twin.crate = b3::create_body(twin.world, &body_def)!!;
    shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    b3::BoxHull cube = b3::make_box_hull(0.5, 0.5, 0.5);
    b3::create_hull_shape(twin.crate, &shape_def, &cube.base)!!;
    for (int i = 0; i < 60; i++) b3::world_step(twin.world, STEP, 4);
    return twin;
}

<* How far the crate's leading face gets past `wall_x` while pushed at 1 m/s for half a second. *>
fn float deepest_past(WallTwin* twin, float wall_x) {
    float deepest = -1.0;
    for (int i = 0; i < 30; i++) {
        twin.crate.set_linear_velocity({1.0, 0.0, 0.0});
        b3::world_step(twin.world, STEP, 4);
        deepest = math::max(deepest, twin.crate.position().x + 0.5f - wall_x);
    }
    return deepest;
}

fn void a_small_shift_leaves_no_stale_mesh_query_behind() @test {
    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 16,
      z_count: 16,
      cell_width: 0.25,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer b3::destroy_mesh(grid);
    WallTwin shifted = a_crate_before_a_mesh_wall(grid);
    defer b3::destroy_world(shifted.world);
    WallTwin reference = a_crate_before_a_mesh_wall(grid);
    defer b3::destroy_world(reference.world);

    // Under the query margin on every axis, so the old world-space query box still holds the
    // crate's new bounds while the triangles cached for it stop short of the wall.
    b3::world_shift_origin(shifted.world, {0.06, 0.0, 0.0});
    float shifted_depth = deepest_past(&shifted, 0.6 - 0.06);
    float reference_depth = deepest_past(&reference, 0.6);
    assert(reference_depth < 0.01, "the unshifted crate stops at the wall's face");
    assert(shifted_depth < reference_depth + 0.002, "and so does the shifted one");
}
