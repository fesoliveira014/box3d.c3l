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
// islands. Wake propagation is the proof, and only against the no-joint control. The nudge
// carries the wake across the island by itself; the step that follows it is incidental.
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

// A jointed pair shares one sleeping solver set and waking a body wakes the whole set, so nothing
// here separates "wakes both" from "wakes either one" — what this pins is the reach of the wake.
fn void waking_a_joint_wakes_both_of_the_bodies_it_links() @test {
    JointWakeOutcome outcome = two_sleeping_jointed_spheres_after_waking_through_the_joint();

    assert(outcome.asleep_a && outcome.asleep_b, "both bodies must be asleep for the wake to mean anything");
    assert(outcome.awake_a && outcome.awake_b, "the wake reaches both bodies the joint links");
}

// A third of a turn about the diagonal. Every component is 0.5, so the quaternion is exactly a
// unit one and the integrator hands it back unchanged on a body nothing is rotating.
const b3::Quat BODY_B_TILT = { .i = 0.5, .j = 0.5, .k = 0.5, .l = 0.5 };

struct JointReaction {
    b3::Vec3 force;
    b3::Vec3 torque;
    float linear_separation;
    float angular_separation;
    b3::Quat rotation_b;
}

fn JointReaction the_reaction_of_a_filter_joint_anchored_off_centre() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);
    b.set_transform(b.position(), BODY_B_TILT);

    b3::FilterJointDef joint_def = b3::default_filter_joint_def();
    joint_def.body_id_a = a;
    joint_def.body_id_b = b;
    joint_def.local_frame_a = ANCHOR_A;
    joint_def.local_frame_b = ANCHOR_B;
    b3::JointId joint = world.create_filter_joint(&joint_def)!!;

    for (int i = 0; i < 8; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return {
        .force = joint.constraint_force(),
        .torque = joint.constraint_torque(),
        .linear_separation = joint.linear_separation(),
        .angular_separation = joint.angular_separation(),
        .rotation_b = b.rotation(),
    };
}

// Each zero is made discriminating by its own half of the setup: the anchors are offset and their
// bodies six apart, so a linear separation read off them would be nowhere near zero, and body B is
// turned a third of the way round, so an angular separation — which box3d takes from the two body
// rotations, never from the anchors — would be nowhere near zero either.
fn void a_filter_joint_reports_no_force_no_torque_and_no_separation() @test {
    JointReaction reaction = the_reaction_of_a_filter_joint_anchored_off_centre();

    assert(reaction.rotation_b.i == BODY_B_TILT.i && reaction.rotation_b.j == BODY_B_TILT.j
        && reaction.rotation_b.k == BODY_B_TILT.k && reaction.rotation_b.l == BODY_B_TILT.l,
        "body B is still turned away from body A when the separations are read");
    assert(reaction.force.x == 0.0 && reaction.force.y == 0.0 && reaction.force.z == 0.0,
        "a joint type that enforces nothing reports no force, whatever its anchors");
    assert(reaction.torque.x == 0.0 && reaction.torque.y == 0.0 && reaction.torque.z == 0.0);
    assert(reaction.linear_separation == 0.0, "and no separation, though its anchors are metres apart");
    assert(reaction.angular_separation == 0.0, "nor any angle, though its bodies are turned apart");
}

struct JointTuning {
    b3::ConstraintTuning from_definition;
    b3::ConstraintTuning after_set;
}

fn JointTuning the_constraint_tuning_of_a_filter_joint() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);
    b3::JointId joint = link_with_a_filter_joint(world, a, b);

    b3::ConstraintTuning from_definition = joint.constraint_tuning();
    joint.set_constraint_tuning(30.0f, 1.5f);

    return {.from_definition = from_definition, .after_set = joint.constraint_tuning()};
}

// 60, 2, 30 and 1.5 are exactly representable as floats, and each is either copied out of the
// definition or written straight back by the setter, so they compare exactly rather than in a band.
fn void a_joint_carries_the_constraint_tuning_its_definition_set_and_takes_a_new_one() @test {
    JointTuning tuning = the_constraint_tuning_of_a_filter_joint();

    assert(tuning.from_definition.hertz == 60.0, "a default definition builds a joint tuned at 60 hertz");
    assert(tuning.from_definition.damping_ratio == 2.0);
    assert(tuning.after_set.hertz == 30.0, "the setter writes both halves of the pair");
    assert(tuning.after_set.damping_ratio == 1.5, "each to its own, so a swap fails");
}

struct JointThresholds {
    float force_before;
    float torque_before;
    float force_after;
    float torque_after;
}

fn JointThresholds the_event_thresholds_of_a_filter_joint() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);
    b3::JointId joint = link_with_a_filter_joint(world, a, b);

    float force_before = joint.force_threshold();
    float torque_before = joint.torque_threshold();
    joint.set_force_threshold(0.0f);
    joint.set_torque_threshold(7.5f);

    return {
        .force_before = force_before,
        .torque_before = torque_before,
        .force_after = joint.force_threshold(),
        .torque_after = joint.torque_threshold(),
    };
}

// float::max, 0 and 7.5 are exactly representable as floats, and each is either copied out of the
// definition or written straight back by the setter, so they compare exactly rather than in a band.
fn void a_joint_holds_both_event_thresholds_out_of_reach_until_they_are_set() @test {
    JointThresholds thresholds = the_event_thresholds_of_a_filter_joint();

    assert(thresholds.force_before == float::max,
        "both thresholds start out of reach, which is what keeps the joint out of the events");
    assert(thresholds.torque_before == float::max);
    assert(thresholds.force_after == 0.0, "a zero threshold arms the check on every awake step");
    assert(thresholds.torque_after == 7.5, "and each threshold takes its own value, so a swap fails");
}

struct BodyJoints {
    int count_fresh;
    int count_with_one_joint;
    int count_with_two_joints;
    int count_after_destroy;
    b3::JointId first;
    b3::JointId second;
    int filled_roomy;
    b3::JointId[4] roomy;
    int filled_cramped;
    b3::JointId[1] cramped;
    int filled_empty;
    b3::JointId[2] untouched;
}

// The buffers are copied out whole rather than returned as slices: a slice out of this helper
// would point into a stack frame that ends with it, so the caller reads the copies and the
// recorded lengths instead.
fn BodyJoints the_joints_of_a_body_linked_to_two_others() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId shared = a_dynamic_sphere_at(world, 0.0);
    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);

    int count_fresh = shared.joint_count();
    b3::JointId first = link_with_a_filter_joint(world, shared, a);
    int count_with_one_joint = shared.joint_count();
    b3::JointId second = link_with_a_filter_joint(world, shared, b);
    int count_with_two_joints = shared.joint_count();

    b3::JointId[4] roomy;
    b3::JointId[1] cramped;
    b3::JointId[2] untouched;
    int filled_roomy = shared.joints(roomy[..]).len;
    int filled_cramped = shared.joints(cramped[..]).len;
    int filled_empty = shared.joints(untouched[:0]).len;

    first.destroy(false);
    second.destroy(false);

    return {
        .count_fresh = count_fresh,
        .count_with_one_joint = count_with_one_joint,
        .count_with_two_joints = count_with_two_joints,
        .count_after_destroy = shared.joint_count(),
        .first = first,
        .second = second,
        .filled_roomy = filled_roomy,
        .roomy = roomy,
        .filled_cramped = filled_cramped,
        .cramped = cramped,
        .filled_empty = filled_empty,
        .untouched = untouched,
    };
}

fn void a_body_counts_the_joints_linked_to_it() @test {
    BodyJoints joints = the_joints_of_a_body_linked_to_two_others();

    assert(joints.count_fresh == 0, "a body carries no joints until one links it");
    assert(joints.count_with_one_joint == 1);
    assert(joints.count_with_two_joints == 2, "and every further joint adds one more");
    assert(joints.count_after_destroy == 0, "destroying both takes the count back to none");
}

fn void a_body_reports_the_joints_linked_to_it_most_recently_created_first() @test {
    BodyJoints joints = the_joints_of_a_body_linked_to_two_others();

    assert(joints.first.index1 != joints.second.index1, "the two joints must differ for the order to mean anything");
    assert(joints.filled_roomy == 2, "a buffer longer than the count comes back as the filled prefix");
    assert(joints.roomy[0].index1 == joints.second.index1,
        "box3d walks a head-inserted list, so the joint created last is reported first");
    assert(joints.roomy[0].generation == joints.second.generation);
    assert(joints.roomy[1].index1 == joints.first.index1, "and the joint created first is reported last");
    assert(joints.roomy[1].generation == joints.first.generation);
}

fn void a_buffer_takes_only_the_joints_that_fit_in_it() @test {
    BodyJoints joints = the_joints_of_a_body_linked_to_two_others();

    assert(joints.filled_cramped == 1, "a buffer of one is filled to its length and no further");
    assert(joints.cramped[0].index1 == joints.second.index1, "and takes the joint the walk reaches first");
    assert(joints.cramped[0].generation == joints.second.generation);
    assert(joints.filled_empty == 0, "a buffer with no room at all takes nothing");
    assert(joints.untouched[0].index1 == 0, "and the buffer it was handed comes back as it went in");
}

struct JointEventReport {
    int count;
    b3::JointId joint;
    b3::JointId reported;
    void* user_data;
}

// The events are read on the step right after the joint is built rather than after a long run:
// box3d reports awake joints only, and this pair falls asleep well inside a hundred steps, so a
// later read finds an empty slice whatever the threshold and passes for the wrong reason. The one
// event is copied out because the slice points into world-owned memory that the next step()
// replaces and destroy() frees.
fn JointEventReport the_joint_events_of_a_filter_joint(bool arm_the_threshold, void* sentinel) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);
    b3::JointId joint = link_with_a_filter_joint(world, a, b);
    joint.set_user_data(sentinel);
    if (arm_the_threshold) joint.set_force_threshold(0.0f);

    world.step(1.0f / 60.0f, 4);

    b3::JointEvent[] events = world.joint_events();
    if (events.len == 0) return {.count = 0, .joint = joint};

    return {
        .count = events.len,
        .joint = joint,
        .reported = events[0].joint_id,
        .user_data = events[0].user_data,
    };
}

// The threshold is the only difference between the two rows. A filter joint's reaction force is
// always zero, and box3d reports a joint when its force reaches the threshold, so zero is the
// only setting a filter joint ever meets — and the default puts both thresholds out of reach.
fn void a_joint_that_reaches_its_force_threshold_is_reported_in_the_joint_events() @test {
    int sentinel = 11;

    JointEventReport armed = the_joint_events_of_a_filter_joint(true, &sentinel);
    assert(armed.count == 1, "an armed joint is reported once for the step it was awake through");
    assert(armed.reported.index1 == armed.joint.index1, "and the event names the joint that reached it");
    assert(armed.reported.generation == armed.joint.generation);
    assert(armed.reported.world0 == armed.joint.world0);

    JointEventReport unarmed = the_joint_events_of_a_filter_joint(false, &sentinel);
    assert(unarmed.count == 0, "a joint holding both thresholds out of reach is reported in nothing");
}

fn void a_joint_event_carries_the_user_data_its_joint_was_given() @test {
    int sentinel = 11;

    JointEventReport armed = the_joint_events_of_a_filter_joint(true, &sentinel);

    assert(armed.count == 1, "the length is what makes the event safe to read");
    assert(armed.user_data == &sentinel, "the event hands back what set_user_data wrote on the joint");
}

fn void the_revolute_joint_types_match_the_c_layout() @test {
    assert(b3::RevoluteJointDef::size == 152);
    assert(b3::RevoluteJointDef::alignment == 8);
}

// 0 is exactly representable as a float, so these compare exactly rather than in a band.
fn void the_default_revolute_joint_definition_zeroes_every_field_it_adds_to_the_base() @test {
    b3::RevoluteJointDef def = b3::default_revolute_joint_def();

    assert(def.target_angle == 0.0);
    assert(!def.enable_spring, "a default revolute joint is a bare hinge");
    assert(def.hertz == 0.0);
    assert(def.damping_ratio == 0.0);
    assert(!def.enable_limit);
    assert(def.lower_angle == 0.0);
    assert(def.upper_angle == 0.0);
    assert(!def.enable_motor);
    assert(def.max_motor_torque == 0.0, "0, not the float::max the base thresholds carry");
    assert(def.motor_speed == 0.0);
}

fn void a_default_revolute_joint_definition_carries_the_cookie_every_definition_shares() @test {
    b3::RevoluteJointDef def = b3::default_revolute_joint_def();
    b3::FilterJointDef filter_def = b3::default_filter_joint_def();

    assert(def.internal_value == def.base.internal_value, "the direct field names the embedded base");
    assert(def.internal_value == filter_def.internal_value, "every definition starts from the same base");
}

fn b3::RevoluteJointId link_with_a_revolute_joint(b3::WorldId world, b3::BodyId a, b3::BodyId b) {
    b3::RevoluteJointDef joint_def = b3::default_revolute_joint_def();
    joint_def.body_id_a = a;
    joint_def.body_id_b = b;
    return world.create_revolute_joint(&joint_def)!!;
}

struct RevoluteHandleReport {
    b3::JointType type;
    b3::BodyId expected_a;
    b3::BodyId reported_a;
    bool valid;
}

// Every accessor here is a shared JointId one called on a RevoluteJointId with no cast, which is
// what the inline typedef buys.
fn RevoluteHandleReport the_shared_accessors_on_a_new_hinge() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -1.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 1.0);
    b3::RevoluteJointId hinge = link_with_a_revolute_joint(world, a, b);

    return {
        .type = hinge.type(),
        .expected_a = a,
        .reported_a = hinge.body_a(),
        .valid = hinge.is_valid(),
    };
}

enum HingeBodies {
    UNSET,
    THIS_WORLD,
    ANOTHER_WORLD,
}

// Two worlds every time, both destroyed, so that the cross-world row differs from the others in
// nothing but which world its second body came from.
fn fault refusal_of_a_revolute_joint_definition(b3::RevoluteJointDef* def, HingeBodies bodies) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();
    b3::WorldId elsewhere = b3::create_world(&world_def)!!;
    defer elsewhere.destroy();

    if (bodies != HingeBodies.UNSET) {
        b3::WorldId home_of_b = bodies == HingeBodies.ANOTHER_WORLD ? elsewhere : world;
        def.body_id_a = a_dynamic_sphere_at(world, -1.0);
        def.body_id_b = a_dynamic_sphere_at(home_of_b, 1.0);
    }

    if (catch err = world.create_revolute_joint(def)) return err;
    return {};
}

struct HingeConversion {
    b3::JointId hinge;
    b3::JointId narrowed;
    bool converted;
    fault filter_refusal;
}

fn HingeConversion the_conversions_of_a_hinge_and_a_filter_joint() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -1.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 1.0);
    b3::JointId hinge = link_with_a_revolute_joint(world, a, b);
    b3::JointId filter = link_with_a_filter_joint(world, a, b);

    HingeConversion conversion = {.hinge = hinge};
    if (try narrowed = hinge.as_revolute()) {
        conversion.narrowed = narrowed;
        conversion.converted = true;
    }
    if (catch err = filter.as_revolute()) conversion.filter_refusal = err;
    return conversion;
}

fn void a_revolute_joint_reports_its_own_type() @test {
    RevoluteHandleReport report = the_shared_accessors_on_a_new_hinge();

    assert(report.type == b3::JointType.REVOLUTE);
}

fn void the_shared_joint_accessors_reach_a_revolute_handle_uncast() @test {
    RevoluteHandleReport report = the_shared_accessors_on_a_new_hinge();

    assert(report.reported_a.index1 == report.expected_a.index1, "body_a() takes the handle as it is");
    assert(report.reported_a.world0 == report.expected_a.world0);
    assert(report.reported_a.generation == report.expected_a.generation);
    assert(report.valid, "and so do type() and is_valid()");
}

fn void a_zeroed_revolute_joint_definition_is_refused() @test {
    b3::RevoluteJointDef zeroed;

    fault refusal = refusal_of_a_revolute_joint_definition(&zeroed, HingeBodies.THIS_WORLD);

    assert(refusal == b3::INVALID_DEFINITION, "box3d accepts a zeroed joint definition; the binding must not");
}

fn void a_revolute_joint_definition_naming_no_bodies_is_refused() @test {
    b3::RevoluteJointDef bodiless = b3::default_revolute_joint_def();

    fault refusal = refusal_of_a_revolute_joint_definition(&bodiless, HingeBodies.UNSET);

    assert(refusal == b3::INVALID_DEFINITION,
        "an unset body identifier indexes outside box3d's body array, which creating the joint writes through");
}

// The control is the row above it: the same definition with both bodies from the calling world is
// accepted, so the refusal here is the world the second body came from and nothing else.
fn void a_revolute_joint_definition_naming_a_body_of_another_world_is_refused() @test {
    b3::RevoluteJointDef stranger = b3::default_revolute_joint_def();

    fault refusal = refusal_of_a_revolute_joint_definition(&stranger, HingeBodies.ANOTHER_WORLD);

    assert(refusal == b3::INVALID_DEFINITION,
        "box3d resolves a body identifier against the world it was handed, not the world it came from");

    fault none;
    b3::RevoluteJointDef local = b3::default_revolute_joint_def();
    fault accepted = refusal_of_a_revolute_joint_definition(&local, HingeBodies.THIS_WORLD);
    assert(accepted == none, "the same definition with both bodies from the calling world is accepted");
}

fn void a_revolute_joint_converts_to_its_own_handle_and_a_filter_joint_does_not() @test {
    HingeConversion conversion = the_conversions_of_a_hinge_and_a_filter_joint();

    assert(conversion.converted, "as_revolute takes back the joint it was given");
    assert(conversion.narrowed.index1 == conversion.hinge.index1);
    assert(conversion.narrowed.world0 == conversion.hinge.world0);
    assert(conversion.narrowed.generation == conversion.hinge.generation);
    assert(conversion.filter_refusal == b3::WRONG_JOINT_TYPE,
        "a revolute accessor on another joint type reinterprets that joint's solver union");
}
