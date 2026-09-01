module b3_test;

import b3;

fn void the_joint_types_match_the_c_layout() @test {
    assert(b3::JointType::size == 4);
    assert(b3::JointDef::size == 112);
    assert(b3::JointDef::alignment == 8);
    assert(b3::FilterJointDef::size == 112);
    assert(b3::FilterJointDef::alignment == 8);
}

fn void the_joint_type_values_match_the_c_enum() @test {
    assert((int)b3::JointType.PARALLEL == 0);
    assert((int)b3::JointType.DISTANCE == 1);
    assert((int)b3::JointType.FILTER == 2);
    assert((int)b3::JointType.MOTOR == 3);
    assert((int)b3::JointType.PRISMATIC == 4);
    assert((int)b3::JointType.REVOLUTE == 5);
    assert((int)b3::JointType.SPHERICAL == 6);
    assert((int)b3::JointType.WELD == 7);
    assert((int)b3::JointType.WHEEL == 8);
}

// 60 and 2 are exactly representable as floats, so they compare exactly rather than in a band.
// draw_scale is not a constant default at all: it is the current length unit.
fn void the_default_filter_joint_definition_carries_the_values_the_c_side_sets() @test {
    b3::FilterJointDef def = b3::default_filter_joint_def();

    assert(def.force_threshold == float::max);
    assert(def.torque_threshold == float::max);
    assert(def.constraint_hertz == 60.0);
    assert(def.constraint_damping_ratio == 2.0);
    assert(def.draw_scale == b3::length_units_per_meter());
    assert(!def.collide_connected);
    assert(def.user_data == null);
}

fn void the_default_filter_joint_definition_names_no_bodies_and_no_offsets() @test {
    b3::FilterJointDef def = b3::default_filter_joint_def();

    assert(def.body_id_a.is_null(), "a definition carries no bodies until the caller sets them");
    assert(def.body_id_b.is_null());
    assert(def.local_frame_a.p.x == 0.0 && def.local_frame_a.p.y == 0.0 && def.local_frame_a.p.z == 0.0);
    assert(def.local_frame_b.p.x == 0.0 && def.local_frame_b.p.y == 0.0 && def.local_frame_b.p.z == 0.0);
    assert(def.local_frame_a.q.i == 0.0 && def.local_frame_a.q.j == 0.0 && def.local_frame_a.q.k == 0.0);
    assert(def.local_frame_b.q.i == 0.0 && def.local_frame_b.q.j == 0.0 && def.local_frame_b.q.k == 0.0);
    assert(def.local_frame_a.q.l == 1.0, "an unrotated frame is the identity quaternion, not a zero one");
    assert(def.local_frame_b.q.l == 1.0);
}

fn void a_filter_joint_definition_reaches_its_base_fields_directly() @test {
    b3::FilterJointDef def = b3::default_filter_joint_def();

    def.force_threshold = 0.0;

    assert(def.base.force_threshold == 0.0, "the direct field names the embedded base");
    assert(def.base.internal_value == def.internal_value);
}

fn void a_zeroed_filter_joint_definition_differs_from_a_default_one() @test {
    b3::FilterJointDef zeroed;
    b3::FilterJointDef def = b3::default_filter_joint_def();

    assert(zeroed.internal_value != def.internal_value);
    assert(zeroed.force_threshold != def.force_threshold, "a zeroed threshold reports an event every step");
}

fn b3::BodyId a_dynamic_sphere_at(b3::WorldId world, float x) {
    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {x, 0.0, 0.0};
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    body.create_sphere_shape(&shape_def, &sphere)!!;
    return body;
}

fn b3::JointId link_with_a_filter_joint(b3::WorldId world, b3::BodyId a, b3::BodyId b) {
    b3::FilterJointDef joint_def = b3::default_filter_joint_def();
    joint_def.body_id_a = a;
    joint_def.body_id_b = b;
    return world.create_filter_joint(&joint_def)!!;
}

enum SphereLinkage {
    UNJOINTED,
    FILTERED,
    FILTERED_BUT_COLLIDING,
}

// The world is destroyed before the caller asserts anything, so a failing assertion cannot
// strand it: C3 skips defer on a panicking assertion and every later world or byte count in the
// run would then measure against a shifted baseline.
fn float[2] two_overlapping_spheres_after_sixty_steps(SphereLinkage linkage) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -0.4);
    b3::BodyId b = a_dynamic_sphere_at(world, 0.4);
    if (linkage != SphereLinkage.UNJOINTED) {
        b3::JointId joint = link_with_a_filter_joint(world, a, b);
        if (linkage == SphereLinkage.FILTERED_BUT_COLLIDING) joint.set_collide_connected(true);
    }

    for (int i = 0; i < 60; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return {a.position().x, b.position().x};
}

fn bool[2] awake_after_nudging_one_of_two_distant_spheres(bool with_joint) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);
    if (with_joint) link_with_a_filter_joint(world, a, b);

    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    a.set_linear_velocity((b3::Vec3){ 1.0, 0.0, 0.0 });
    world.step(1.0f / 60.0f, 4);

    return {a.is_awake(), b.is_awake()};
}

struct DestroyedJointOutcome {
    bool asleep_before;
    bool valid_before;
    bool valid_after;
    bool awake_a;
    bool awake_b;
}

fn DestroyedJointOutcome two_sleeping_jointed_spheres_after_destroy(bool wake_attached) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);
    b3::JointId joint = link_with_a_filter_joint(world, a, b);

    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    bool asleep_before = !a.is_awake() && !b.is_awake();
    bool valid_before = joint.is_valid();
    joint.destroy(wake_attached);

    return {
        .asleep_before = asleep_before,
        .valid_before = valid_before,
        .valid_after = joint.is_valid(),
        .awake_a = a.is_awake(),
        .awake_b = b.is_awake(),
    };
}

// The refusal comes back as a plain value for the same reason the helpers above return one: the
// world is finished before the caller asserts, so a failing assertion cannot strand it.
fn fault refusal_of_a_filter_joint_definition(b3::FilterJointDef* def) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    if (catch err = world.create_filter_joint(def)) return err;
    return {};
}

// Two spheres of radius 0.5 whose centres start 0.8 apart overlap by 0.2, so they separate until
// they touch at 1.0 unless the pair is filtered out of collision. The control row is what makes
// the jointed row mean anything.
fn void a_filter_joint_stops_two_overlapping_bodies_pushing_apart() @test {
    float[2] unjointed = two_overlapping_spheres_after_sixty_steps(SphereLinkage.UNJOINTED);
    float gap = unjointed[1] - unjointed[0];
    assert(gap > 0.999 && gap < 1.001, "left to collide, the two spheres push apart until they touch");

    float[2] jointed = two_overlapping_spheres_after_sixty_steps(SphereLinkage.FILTERED);
    float filtered_gap = jointed[1] - jointed[0];
    assert(filtered_gap > 0.7995 && filtered_gap < 0.8005,
        "a filter joint leaves them overlapping exactly where they started");
    assert(jointed[0] > -0.4005 && jointed[0] < -0.3995, "and neither body moves at all");
    assert(jointed[1] > 0.3995 && jointed[1] < 0.4005);
}

// Both pairs fall asleep whether or not a joint links them, so sleeping proves nothing about
// islands. Wake propagation is the proof, and only against the no-joint control.
fn void a_filter_joint_keeps_the_two_bodies_in_one_island() @test {
    bool[2] jointed = awake_after_nudging_one_of_two_distant_spheres(true);
    assert(jointed[0], "the nudged body wakes");
    assert(jointed[1], "and the joint carries the wake across to the other body");

    bool[2] unjointed = awake_after_nudging_one_of_two_distant_spheres(false);
    assert(unjointed[0], "the nudged body wakes without a joint too");
    assert(!unjointed[1], "but nothing carries the wake to a body in another island");
}

fn void the_destroy_flag_decides_whether_the_attached_bodies_wake() @test {
    DestroyedJointOutcome left_asleep = two_sleeping_jointed_spheres_after_destroy(false);
    assert(left_asleep.asleep_before, "the pair must be asleep for the flag to mean anything");
    assert(!left_asleep.awake_a && !left_asleep.awake_b, "destroying without waking leaves both asleep");

    DestroyedJointOutcome woken = two_sleeping_jointed_spheres_after_destroy(true);
    assert(woken.asleep_before);
    assert(woken.awake_a && woken.awake_b, "destroying with waking wakes both");
}

fn void a_joint_is_valid_until_it_is_destroyed() @test {
    DestroyedJointOutcome outcome = two_sleeping_jointed_spheres_after_destroy(false);

    assert(outcome.valid_before);
    assert(!outcome.valid_after, "a destroyed joint must not report itself valid");
}

fn void a_zeroed_filter_joint_definition_is_refused() @test {
    b3::FilterJointDef zeroed;

    fault refusal = refusal_of_a_filter_joint_definition(&zeroed);

    assert(refusal == b3::INVALID_DEFINITION, "box3d accepts a zeroed joint definition; the binding must not");
}

fn void a_filter_joint_definition_naming_no_bodies_is_refused() @test {
    b3::FilterJointDef bodiless = b3::default_filter_joint_def();

    fault refusal = refusal_of_a_filter_joint_definition(&bodiless);

    assert(refusal == b3::INVALID_DEFINITION,
        "an unset body identifier indexes outside box3d's body array, which creating the joint writes through");
}

struct JointIdentity {
    b3::JointType type;
    b3::BodyId a;
    b3::BodyId b;
    b3::BodyId reported_a;
    b3::BodyId reported_b;
    b3::WorldId world;
    b3::WorldId reported_world;
}

fn JointIdentity the_identity_of_a_filter_joint() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);
    b3::JointId joint = link_with_a_filter_joint(world, a, b);

    return {
        .type = joint.type(),
        .a = a,
        .b = b,
        .reported_a = joint.body_a(),
        .reported_b = joint.body_b(),
        .world = world,
        .reported_world = joint.world(),
    };
}

fn void a_filter_joint_reports_the_filter_type() @test {
    JointIdentity identity = the_identity_of_a_filter_joint();

    assert(identity.type == b3::JointType.FILTER);
}

fn void a_joint_reports_its_two_bodies_in_the_order_its_definition_named_them() @test {
    JointIdentity identity = the_identity_of_a_filter_joint();

    assert(identity.a.index1 != identity.b.index1, "the two bodies must differ for the order to mean anything");
    assert(identity.reported_a.index1 == identity.a.index1, "body A is the body the definition named first");
    assert(identity.reported_a.generation == identity.a.generation);
    assert(identity.reported_b.index1 == identity.b.index1, "and body B the one it named second");
    assert(identity.reported_b.generation == identity.b.generation);
}

fn void a_joint_knows_which_world_holds_it() @test {
    JointIdentity identity = the_identity_of_a_filter_joint();

    assert(identity.reported_world.index1 == identity.world.index1);
    assert(identity.reported_world.generation == identity.world.generation);
}

// Each anchor's quaternion reuses its own position components so that a transposed field fails the
// round trip; l is whatever is left to make the four of them a unit quaternion.
const b3::Transform ANCHOR_A = {
    .p = { 0.25, 0.5, 0.75 },
    .q = { .i = 0.25, .j = 0.5, .k = 0.75, .l = 0.35355339 },
};

const b3::Transform ANCHOR_B = {
    .p = { -0.25, -0.5, -0.75 },
    .q = { .i = -0.25, .j = -0.5, .k = -0.75, .l = 0.35355339 },
};

struct JointFrames {
    b3::Transform from_definition_a;
    b3::Transform from_definition_b;
    b3::Transform after_setters_a;
    b3::Transform after_setters_b;
}

fn JointFrames the_local_frames_of_a_filter_joint() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);

    b3::FilterJointDef joint_def = b3::default_filter_joint_def();
    joint_def.body_id_a = a;
    joint_def.body_id_b = b;
    joint_def.local_frame_a = ANCHOR_A;
    joint_def.local_frame_b = ANCHOR_B;
    b3::JointId joint = world.create_filter_joint(&joint_def)!!;

    b3::Transform from_definition_a = joint.local_frame_a();
    b3::Transform from_definition_b = joint.local_frame_b();

    joint.set_local_frame_a(ANCHOR_B);
    joint.set_local_frame_b(ANCHOR_A);

    return {
        .from_definition_a = from_definition_a,
        .from_definition_b = from_definition_b,
        .after_setters_a = joint.local_frame_a(),
        .after_setters_b = joint.local_frame_b(),
    };
}

fn bool a_frame_matches(b3::Transform got, b3::Transform want) {
    return got.p.x == want.p.x && got.p.y == want.p.y && got.p.z == want.p.z
        && got.q.i == want.q.i && got.q.j == want.q.j && got.q.k == want.q.k && got.q.l == want.q.l;
}

fn void a_joint_reports_the_local_frames_its_definition_carried() @test {
    JointFrames frames = the_local_frames_of_a_filter_joint();

    assert(a_frame_matches(frames.from_definition_a, ANCHOR_A), "the anchor comes back rotation and all");
    assert(a_frame_matches(frames.from_definition_b, ANCHOR_B), "and body B's anchor is its own, not body A's");
}

fn void a_joint_local_frame_survives_a_setter_round_trip() @test {
    JointFrames frames = the_local_frames_of_a_filter_joint();

    assert(a_frame_matches(frames.after_setters_a, ANCHOR_B), "the setter writes the anchor it names");
    assert(a_frame_matches(frames.after_setters_b, ANCHOR_A), "each of them its own body's, so a swap fails");
}

fn bool[2] collision_between_two_jointed_bodies_before_and_after_putting_it_back() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);
    b3::JointId joint = link_with_a_filter_joint(world, a, b);

    bool before = joint.collide_connected();
    joint.set_collide_connected(true);

    return {before, joint.collide_connected()};
}

fn void a_joint_forbids_collision_between_its_bodies_until_the_flag_puts_it_back() @test {
    bool[2] allowed = collision_between_two_jointed_bodies_before_and_after_putting_it_back();

    assert(!allowed[0], "a default definition builds a joint that keeps the two bodies apart");
    assert(allowed[1], "and the setter puts the collision back");
}

// The flag, not the joint's type, is what suppressed the collision: the joint is still there and
// the pair lands exactly where the unjointed control does.
fn void putting_collision_back_lets_two_jointed_bodies_push_apart_again() @test {
    float[2] colliding = two_overlapping_spheres_after_sixty_steps(SphereLinkage.FILTERED_BUT_COLLIDING);
    float gap = colliding[1] - colliding[0];

    assert(gap > 0.999 && gap < 1.001, "with the flag set the two spheres push apart until they touch");
}

struct JointUserData {
    void* before;
    void* after;
}

fn JointUserData the_user_data_of_a_filter_joint(void* sentinel) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);
    b3::JointId joint = link_with_a_filter_joint(world, a, b);

    void* before = joint.user_data();
    joint.set_user_data(sentinel);

    return {.before = before, .after = joint.user_data()};
}

fn void user_data_round_trips_on_a_joint() @test {
    int sentinel = 7;

    JointUserData carried = the_user_data_of_a_filter_joint(&sentinel);

    assert(carried.before == null, "a joint carries the definition's user data, which names nothing by default");
    assert(carried.after == &sentinel);
}

struct JointWakeOutcome {
    bool asleep_a;
    bool asleep_b;
    bool awake_a;
    bool awake_b;
}

fn JointWakeOutcome two_sleeping_jointed_spheres_after_waking_through_the_joint() {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);
    b3::JointId joint = link_with_a_filter_joint(world, a, b);

    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    bool asleep_a = !a.is_awake();
    bool asleep_b = !b.is_awake();
    joint.wake_bodies();

    return {
        .asleep_a = asleep_a,
        .asleep_b = asleep_b,
        .awake_a = a.is_awake(),
        .awake_b = b.is_awake(),
    };
}

fn void waking_a_joint_wakes_both_of_the_bodies_it_links() @test {
    JointWakeOutcome outcome = two_sleeping_jointed_spheres_after_waking_through_the_joint();

    assert(outcome.asleep_a && outcome.asleep_b, "both bodies must be asleep for the wake to mean anything");
    assert(outcome.awake_a && outcome.awake_b, "the joint wakes both bodies, not only one");
}
