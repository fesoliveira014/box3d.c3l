module b3_test;

import b3;
import std::math;

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

struct RecycledWorldOutcome {
    bool same_slot;
    bool not_null;
    fault refusal;
}

// A body identifier outlives the world it came from, and box3d hands the next world the slot the
// destroyed one left. The dead world gets two bodies and the live one gets a single body, so the
// stale identifier names the live world's slot at an index that world never issued, with both of
// the fields a hand-written guard can compare still matching. The first world is scoped so that
// its defer destroys it before the second is created, which is what hands the slot on.
fn RecycledWorldOutcome the_refusal_of_a_definition_naming_a_recycled_world() {
    b3::WorldDef world_def = b3::default_world_def();

    b3::BodyId stale;
    {
        b3::WorldId gone = b3::create_world(&world_def)!!;
        defer gone.destroy();
        a_dynamic_sphere_at(gone, -1.0);
        stale = a_dynamic_sphere_at(gone, 1.0);
    }

    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();
    b3::BodyId live = a_dynamic_sphere_at(world, 0.0);

    b3::FilterJointDef joint_def = b3::default_filter_joint_def();
    joint_def.body_id_a = live;
    joint_def.body_id_b = stale;

    RecycledWorldOutcome outcome = {
        .same_slot = stale.world0 == world.index1 - 1,
        .not_null = !stale.is_null(),
    };
    if (catch err = world.create_filter_joint(&joint_def)) outcome.refusal = err;
    return outcome;
}

fn void a_joint_definition_naming_a_body_of_a_destroyed_world_is_refused() @test {
    RecycledWorldOutcome outcome = the_refusal_of_a_definition_naming_a_recycled_world();

    assert(outcome.same_slot, "the recycled world took the destroyed one's slot, so the world comparison passes");
    assert(outcome.not_null, "and the identifier is set, so the null comparison passes too");
    assert(outcome.refusal == b3::INVALID_DEFINITION,
        "a body identifier the live world never issued is indexed and written through all the same");
}

struct RecycledWorldHandleOutcome {
    bool same_slot;
    bool bodies_valid;
    fault refusal;
}

// The mirror of the case above: the two bodies are live and it is the world handle that is stale.
// box3d's generation test on a world identifier is an assertion this release build compiles out,
// so the constructor would resolve the destroyed world's handle to the world that has since taken
// its slot and build the joint in that one instead. The first world exists only long enough to
// hand its slot on, so only its handle outlives it and the world itself is scoped.
fn RecycledWorldHandleOutcome the_refusal_of_a_joint_built_through_a_recycled_world_handle() {
    b3::WorldDef world_def = b3::default_world_def();

    b3::WorldId gone;
    {
        gone = b3::create_world(&world_def)!!;
        defer gone.destroy();
    }

    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();
    b3::BodyId a = a_dynamic_sphere_at(world, -1.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 1.0);

    b3::FilterJointDef joint_def = b3::default_filter_joint_def();
    joint_def.body_id_a = a;
    joint_def.body_id_b = b;

    RecycledWorldHandleOutcome outcome = {
        .same_slot = gone.index1 == world.index1,
        .bodies_valid = a.is_valid() && b.is_valid(),
    };
    if (catch err = gone.create_filter_joint(&joint_def)) outcome.refusal = err;
    return outcome;
}

fn void a_joint_built_through_a_handle_to_a_destroyed_world_is_refused() @test {
    RecycledWorldHandleOutcome outcome = the_refusal_of_a_joint_built_through_a_recycled_world_handle();

    assert(outcome.same_slot, "the live world took the destroyed one's slot, so the world comparison passes");
    assert(outcome.bodies_valid, "and both bodies are live, so the identifier checks pass too");
    assert(outcome.refusal == b3::INVALID_DEFINITION,
        "only the world's own generation separates the stale handle from the live world it names");
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

struct DestroyedHingeConversion {
    bool valid_before;
    b3::JointType type_after;
    fault refusal;
}

// The joint is destroyed while its world lives on, so the handle names a joint slot box3d has
// freed rather than a world that is gone. box3d leaves the slot's type where it was, which is why
// reading the type is not on its own enough to refuse the handle.
fn DestroyedHingeConversion the_conversion_of_a_destroyed_hinge() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -1.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 1.0);
    b3::JointId hinge = link_with_a_revolute_joint(world, a, b);

    DestroyedHingeConversion conversion = {.valid_before = hinge.is_valid()};
    hinge.destroy(false);
    conversion.type_after = hinge.type();
    if (catch err = hinge.as_revolute()) conversion.refusal = err;
    return conversion;
}

fn void a_destroyed_joint_does_not_narrow_to_a_revolute_handle() @test {
    DestroyedHingeConversion conversion = the_conversion_of_a_destroyed_hinge();

    assert(conversion.valid_before, "the joint was live before it was destroyed");
    assert(conversion.type_after == b3::JointType.REVOLUTE,
        "the freed slot still reports the type it held, so the type test alone would hand the handle back");
    assert(conversion.refusal == b3::INVALID_HANDLE,
        "a handle to a joint that no longer exists is refused before its type is read");
}

fn b3::BodyId a_static_sphere_at_the_origin(b3::WorldId world) {
    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    body.create_sphere_shape(&shape_def, &sphere)!!;
    return body;
}

struct HingeDrive {
    float angle;
    b3::Quat rotation;
}

// One second at 1/60 s, against a static body, so the whole rotation is the dynamic body's and
// the motor is the only thing acting on it.
fn HingeDrive a_hinge_after_a_second(bool enable_motor) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId turning = a_dynamic_sphere_at(world, 1.0);

    b3::RevoluteJointDef joint_def = b3::default_revolute_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = turning;
    joint_def.enable_motor = enable_motor;
    joint_def.motor_speed = 2.0;
    joint_def.max_motor_torque = 1000.0;
    b3::RevoluteJointId hinge = world.create_revolute_joint(&joint_def)!!;

    for (int i = 0; i < 60; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return {.angle = hinge.angle(), .rotation = turning.rotation()};
}

struct MotorSettings {
    bool enabled;
    float speed;
    float max_torque;
    bool enabled_after;
    float speed_after;
    float max_torque_after;
}

fn MotorSettings the_motor_settings_of_a_hinge_before_and_after_the_setters() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId turning = a_dynamic_sphere_at(world, 1.0);

    b3::RevoluteJointDef joint_def = b3::default_revolute_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = turning;
    joint_def.enable_motor = true;
    joint_def.motor_speed = 2.0;
    joint_def.max_motor_torque = 1000.0;
    b3::RevoluteJointId hinge = world.create_revolute_joint(&joint_def)!!;

    MotorSettings settings = {
        .enabled = hinge.is_motor_enabled(),
        .speed = hinge.motor_speed(),
        .max_torque = hinge.max_motor_torque(),
    };

    hinge.enable_motor(false);
    hinge.set_motor_speed(-0.5);
    hinge.set_max_motor_torque(12.5);

    settings.enabled_after = hinge.is_motor_enabled();
    settings.speed_after = hinge.motor_speed();
    settings.max_torque_after = hinge.max_motor_torque();
    return settings;
}

struct HingeTorque {
    float motor_torque;
    float constraint_torque_z;
}

// The arm weighs far more than a 500 newton-metre motor can hold at this length, so the motor
// stays at its ceiling for the whole run instead of settling at a load it balances. That is what
// makes the reading exact enough to compare the two torques against each other.
fn HingeTorque the_torques_of_an_arm_heavier_than_its_motor() {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, -10.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId arm = world.create_body(&body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1000.0;
    b3::Capsule capsule = {.center1 = {0.0, 0.0, 0.0}, .center2 = {2.0, 0.0, 0.0}, .radius = 0.25};
    arm.create_capsule_shape(&shape_def, &capsule)!!;

    b3::RevoluteJointDef joint_def = b3::default_revolute_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = arm;
    joint_def.enable_motor = true;
    joint_def.motor_speed = 0.0;
    joint_def.max_motor_torque = 500.0;
    b3::RevoluteJointId hinge = world.create_revolute_joint(&joint_def)!!;

    for (int i = 0; i < 180; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return {.motor_torque = hinge.motor_torque(), .constraint_torque_z = hinge.constraint_torque().z};
}

// 1.8994 rather than the nominal 2.0 because the solver ramps the motor up over the first steps.
// The band is wide enough to hold that and narrow enough to exclude both a hinge that never
// turned and one that reached the nominal figure.
fn void a_motor_turns_the_hinge_at_about_the_speed_it_was_given() @test {
    HingeDrive driven = a_hinge_after_a_second(true);

    assert(driven.angle > 1.85 && driven.angle < 1.95,
        "a motor at 2 rad/s turns the hinge nearly two radians in a second");
}

fn void a_motor_turns_the_hinge_about_the_frames_z_axis_and_no_other() @test {
    HingeDrive driven = a_hinge_after_a_second(true);

    assert(driven.rotation.i > -0.001 && driven.rotation.i < 0.001,
        "an axis confusion turns a plausible angle about the wrong axis");
    assert(driven.rotation.j > -0.001 && driven.rotation.j < 0.001);
    assert(driven.rotation.k > 0.80 && driven.rotation.k < 0.83);
}

// The control for the two above: the same definition with only the motor flag cleared.
fn void a_hinge_whose_motor_is_disabled_does_not_turn() @test {
    HingeDrive idle = a_hinge_after_a_second(false);

    assert(idle.angle > -0.01 && idle.angle < 0.01, "the speed and torque a disabled motor carries drive nothing");
}

fn void the_motor_settings_of_a_hinge_round_trip() @test {
    MotorSettings settings = the_motor_settings_of_a_hinge_before_and_after_the_setters();

    assert(settings.enabled, "the joint reports the motor its definition asked for");
    assert(settings.speed == 2.0);
    assert(settings.max_torque == 1000.0);
    assert(!settings.enabled_after, "and then the values the setters gave it");
    assert(settings.speed_after == -0.5);
    assert(settings.max_torque_after == 12.5);
}

// An upstream trait a consumer reaching for the shared accessor would otherwise be caught by:
// box3d adds the hinge's axial impulse to the constraint torque twice about the same axis, so
// only the per-type reading is the torque the motor applied.
fn void the_shared_constraint_torque_reports_twice_a_revolute_motor_torque() @test {
    HingeTorque torque = the_torques_of_an_arm_heavier_than_its_motor();

    assert(torque.motor_torque > 499.9 && torque.motor_torque < 500.1,
        "the motor is held at the ceiling it was given");
    assert(torque.constraint_torque_z > 999.8 && torque.constraint_torque_z < 1000.2);

    float ratio = torque.constraint_torque_z / torque.motor_torque;
    assert(ratio > 1.999 && ratio < 2.001, "the shared accessor double-counts the hinge axis");
}

// The same motor as the drive above with a limit in its definition, run for twice as long as an
// unlimited hinge needs to pass 0.5, so the reading is the limit holding rather than the run
// stopping early.
fn float the_angle_of_a_motor_driven_hinge_after_two_seconds(bool enable_limit) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId turning = a_dynamic_sphere_at(world, 1.0);

    b3::RevoluteJointDef joint_def = b3::default_revolute_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = turning;
    joint_def.enable_motor = true;
    joint_def.motor_speed = 2.0;
    joint_def.max_motor_torque = 1000.0;
    joint_def.enable_limit = enable_limit;
    joint_def.lower_angle = -0.5;
    joint_def.upper_angle = 0.5;
    b3::RevoluteJointId hinge = world.create_revolute_joint(&joint_def)!!;

    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return hinge.angle();
}

struct HingeLimits {
    float lower;
    float upper;
}

fn HingeLimits the_limits_of_a_hinge_set_to(float lower, float upper) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId turning = a_dynamic_sphere_at(world, 1.0);
    b3::RevoluteJointId hinge = link_with_a_revolute_joint(world, anchor, turning);

    hinge.set_limits(lower, upper);
    return {.lower = hinge.lower_limit(), .upper = hinge.upper_limit()};
}

fn HingeLimits the_limits_of_a_hinge_whose_definition_reversed_them() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId turning = a_dynamic_sphere_at(world, 1.0);

    b3::RevoluteJointDef joint_def = b3::default_revolute_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = turning;
    joint_def.enable_limit = true;
    joint_def.lower_angle = 1.0;
    joint_def.upper_angle = -1.0;
    b3::RevoluteJointId hinge = world.create_revolute_joint(&joint_def)!!;

    return {.lower = hinge.lower_limit(), .upper = hinge.upper_limit()};
}

struct HingeLimitFlags {
    bool on_a_default_hinge;
    bool from_the_definition;
    bool after_enabling;
    bool after_disabling;
}

fn HingeLimitFlags the_limit_flags_of_a_bare_hinge_and_a_limited_one() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId turning = a_dynamic_sphere_at(world, 1.0);
    b3::RevoluteJointId bare = link_with_a_revolute_joint(world, anchor, turning);

    b3::RevoluteJointDef joint_def = b3::default_revolute_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = turning;
    joint_def.enable_limit = true;
    joint_def.lower_angle = -0.5;
    joint_def.upper_angle = 0.5;
    b3::RevoluteJointId limited = world.create_revolute_joint(&joint_def)!!;

    HingeLimitFlags flags = {
        .on_a_default_hinge = bare.is_limit_enabled(),
        .from_the_definition = limited.is_limit_enabled(),
    };

    bare.enable_limit(true);
    limited.enable_limit(false);
    flags.after_enabling = bare.is_limit_enabled();
    flags.after_disabling = limited.is_limit_enabled();
    return flags;
}

struct HingeSeparations {
    float while_the_limit_is_off;
    float once_the_limit_is_exceeded;
}

// The hinge is driven well past the limit first and the limit armed afterwards without a further
// step, so the two readings differ in the limit's state and in nothing else.
fn HingeSeparations the_angular_separations_of_a_turned_hinge_around_arming_its_limit() {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId turning = a_dynamic_sphere_at(world, 1.0);

    b3::RevoluteJointDef joint_def = b3::default_revolute_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = turning;
    joint_def.enable_motor = true;
    joint_def.motor_speed = 2.0;
    joint_def.max_motor_torque = 1000.0;
    b3::RevoluteJointId hinge = world.create_revolute_joint(&joint_def)!!;

    for (int i = 0; i < 60; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    float off = hinge.angular_separation();
    hinge.set_limits(-0.1, 0.1);
    hinge.enable_limit(true);

    return {.while_the_limit_is_off = off, .once_the_limit_is_exceeded = hinge.angular_separation()};
}

// The band holds 0.500134 and excludes both a hinge the limit never caught and the angle the same
// motor reaches with the limit cleared.
fn void a_limit_stops_the_motor_that_drives_the_hinge_into_it() @test {
    float angle = the_angle_of_a_motor_driven_hinge_after_two_seconds(true);

    assert(angle > 0.49 && angle < 0.51, "the motor is held at the limit rather than turning past it");
}

// The control for the one above: the same definition with only the limit flag cleared. Two seconds
// at 2 rad/s carries the hinge past half a turn and angle() is kept in [-pi, pi], so the unlimited
// outcome is the 1.899400 the motor test measures over the first second plus another two radians,
// wrapped. The band holds the measured -2.383774 and excludes both the 0.500134 the limit holds
// and a hinge that never turned.
fn void a_hinge_whose_limit_is_disabled_turns_past_it() @test {
    float angle = the_angle_of_a_motor_driven_hinge_after_two_seconds(false);

    assert(angle > -2.44 && angle < -2.34, "the two angles a disabled limit carries hold nothing back");
}

// 1 and -1 are exactly representable and need no clamping, so the swap shows up exactly.
fn void a_reversed_pair_of_limits_is_swapped_into_order_rather_than_refused() @test {
    HingeLimits limits = the_limits_of_a_hinge_set_to(1.0, -1.0);

    assert(limits.lower == -1.0, "box3d takes the smaller of the two as the lower");
    assert(limits.upper == 1.0);
}

fn void limits_beyond_the_half_turn_box3d_allows_are_clamped_to_it() @test {
    HingeLimits limits = the_limits_of_a_hinge_set_to(-10.0, 10.0);

    assert(limits.lower > -3.12 && limits.lower < -3.10, "0.99 pi, not the ten radians it was handed");
    assert(limits.upper > 3.10 && limits.upper < 3.12);
}

// The constructor applies the same correction as set_limits, so a definition can name its two
// angles in either order and the joint still comes out ordered.
fn void a_definition_that_reverses_its_two_angles_still_builds_an_ordered_limit() @test {
    HingeLimits limits = the_limits_of_a_hinge_whose_definition_reversed_them();

    assert(limits.lower == -1.0);
    assert(limits.upper == 1.0);
}

fn void the_limit_flag_of_a_hinge_starts_off_and_round_trips() @test {
    HingeLimitFlags flags = the_limit_flags_of_a_bare_hinge_and_a_limited_one();

    assert(!flags.on_a_default_hinge, "a default revolute joint is a bare hinge");
    assert(flags.from_the_definition, "the joint reports the limit its definition asked for");
    assert(flags.after_enabling);
    assert(!flags.after_disabling);
}

// The concrete case of a shared accessor whose zero is the joint's state rather than its type:
// the same hinge at the same angle reads zero with the limit off and the full quaternion angle
// once the limit is armed inside it.
fn void a_hinge_reports_no_angular_separation_until_its_limit_is_armed() @test {
    HingeSeparations separations = the_angular_separations_of_a_turned_hinge_around_arming_its_limit();

    assert(separations.while_the_limit_is_off == 0.0, "box3d strips the hinge angle while the limit is off");
    assert(separations.once_the_limit_is_exceeded > 1.85 && separations.once_the_limit_is_exceeded < 1.95,
        "the zero was the limit's state, not a separation this joint type never measures");
}

// Five seconds at 1/60 s against a static body with no gravity and no motor, so the spring is
// the only thing that can turn the hinge and it has time to settle rather than still be swinging.
fn float the_angle_of_a_sprung_hinge_after_five_seconds(bool enable_spring, float target) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId turning = a_dynamic_sphere_at(world, 1.0);

    b3::RevoluteJointDef joint_def = b3::default_revolute_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = turning;
    joint_def.enable_spring = enable_spring;
    joint_def.hertz = 2.0;
    joint_def.damping_ratio = 1.0;
    joint_def.target_angle = target;
    b3::RevoluteJointId hinge = world.create_revolute_joint(&joint_def)!!;

    for (int i = 0; i < 300; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return hinge.angle();
}

struct SpringSettings {
    bool enabled;
    float hertz;
    float damping_ratio;
    float target;
    bool enabled_after;
    float hertz_after;
    float damping_ratio_after;
    float target_after;
}

fn SpringSettings the_spring_settings_of_a_hinge_before_and_after_the_setters() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId turning = a_dynamic_sphere_at(world, 1.0);

    b3::RevoluteJointDef joint_def = b3::default_revolute_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = turning;
    joint_def.enable_spring = true;
    joint_def.hertz = 2.0;
    joint_def.damping_ratio = 1.0;
    joint_def.target_angle = 0.5;
    b3::RevoluteJointId hinge = world.create_revolute_joint(&joint_def)!!;

    SpringSettings settings = {
        .enabled = hinge.is_spring_enabled(),
        .hertz = hinge.spring_hertz(),
        .damping_ratio = hinge.spring_damping_ratio(),
        .target = hinge.target_angle(),
    };

    hinge.enable_spring(false);
    hinge.set_spring_hertz(8.0);
    hinge.set_spring_damping_ratio(0.25);
    hinge.set_target_angle(-1.5);

    settings.enabled_after = hinge.is_spring_enabled();
    settings.hertz_after = hinge.spring_hertz();
    settings.damping_ratio_after = hinge.spring_damping_ratio();
    settings.target_after = hinge.target_angle();
    return settings;
}

// The band holds the measured 0.699938 and excludes a hinge the spring never moved.
fn void a_spring_settles_the_hinge_on_its_target_angle() @test {
    float angle = the_angle_of_a_sprung_hinge_after_five_seconds(true, 0.7);

    assert(angle > 0.69 && angle < 0.71, "a critically damped spring arrives on its target and stays there");
}

// The same spring aimed the other way, so the test pins the target rather than one settled angle.
fn void a_spring_settles_the_hinge_wherever_its_target_points() @test {
    float angle = the_angle_of_a_sprung_hinge_after_five_seconds(true, -0.4);

    assert(angle > -0.41 && angle < -0.39, "the hinge follows the target rather than a fixed angle");
}

// The control for the two above: the same stiffness, damping and target with only the spring flag
// cleared. Without it a settle test could pass on a hinge that was never sprung at all.
fn void a_hinge_whose_spring_is_disabled_does_not_turn() @test {
    float angle = the_angle_of_a_sprung_hinge_after_five_seconds(false, 0.7);

    assert(angle > -0.01 && angle < 0.01, "the stiffness and target a disabled spring carries pull on nothing");
}

// 2, 1, 0.5, 8, 0.25 and -1.5 are all exactly representable as floats, so they compare exactly.
fn void the_spring_settings_of_a_hinge_round_trip() @test {
    SpringSettings settings = the_spring_settings_of_a_hinge_before_and_after_the_setters();

    assert(settings.enabled, "the joint reports the spring its definition asked for");
    assert(settings.hertz == 2.0);
    assert(settings.damping_ratio == 1.0);
    assert(settings.target == 0.5);
    assert(!settings.enabled_after, "and then the values the setters gave it");
    assert(settings.hertz_after == 8.0);
    assert(settings.damping_ratio_after == 0.25);
    assert(settings.target_after == -1.5);
}

fn void the_distance_joint_types_match_the_c_layout() @test {
    assert(b3::DistanceJointDef::size == 160);
    assert(b3::DistanceJointDef::alignment == 8);
}

// Unlike the revolute default, this one does not zero the fields it adds to the base: four of the
// twelve arrive set. 1, 0 and float::max are exactly representable as floats and are copied
// straight out of the definition, so they compare exactly rather than in a band. max_length is
// box3d's huge value, which is 1.0e5 scaled by the length unit rather than a constant, so it is
// compared against that scaling and not against a literal.
fn void the_default_distance_joint_definition_carries_the_four_values_it_does_not_zero() @test {
    b3::DistanceJointDef def = b3::default_distance_joint_def();

    assert(def.length == 1.0, "a default distance joint holds its bodies one length unit apart");
    assert(def.lower_spring_force == -float::max);
    assert(def.upper_spring_force == float::max);
    assert(def.max_length == 1.0e5f * b3::length_units_per_meter());
}

fn void the_default_distance_joint_definition_zeroes_the_other_eight() @test {
    b3::DistanceJointDef def = b3::default_distance_joint_def();

    assert(!def.enable_spring, "a default distance joint is rigid");
    assert(def.hertz == 0.0);
    assert(def.damping_ratio == 0.0);
    assert(!def.enable_limit);
    assert(def.min_length == 0.0);
    assert(!def.enable_motor);
    assert(def.max_motor_force == 0.0, "0, not the float::max the base thresholds carry");
    assert(def.motor_speed == 0.0);
}

fn void a_default_distance_joint_definition_carries_the_cookie_every_definition_shares() @test {
    b3::DistanceJointDef def = b3::default_distance_joint_def();
    b3::FilterJointDef filter_def = b3::default_filter_joint_def();

    assert(def.internal_value == def.base.internal_value, "the direct field names the embedded base");
    assert(def.internal_value == filter_def.internal_value, "every definition starts from the same base");
}

fn b3::DistanceJointId link_with_a_distance_joint(b3::WorldId world, b3::BodyId a, b3::BodyId b) {
    b3::DistanceJointDef joint_def = b3::default_distance_joint_def();
    joint_def.body_id_a = a;
    joint_def.body_id_b = b;
    return world.create_distance_joint(&joint_def)!!;
}

struct DistanceHandleReport {
    b3::JointType type;
    b3::BodyId expected_a;
    b3::BodyId reported_a;
    bool valid;
}

// Every accessor here is a shared JointId one called on a DistanceJointId with no cast, which is
// what the inline typedef buys.
fn DistanceHandleReport the_shared_accessors_on_a_new_rope() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -1.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 1.0);
    b3::DistanceJointId rope = link_with_a_distance_joint(world, a, b);

    return {
        .type = rope.type(),
        .expected_a = a,
        .reported_a = rope.body_a(),
        .valid = rope.is_valid(),
    };
}

fn fault refusal_of_a_distance_joint_definition(b3::DistanceJointDef* def, bool name_bodies) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    if (name_bodies) {
        def.body_id_a = a_dynamic_sphere_at(world, -1.0);
        def.body_id_b = a_dynamic_sphere_at(world, 1.0);
    }

    if (catch err = world.create_distance_joint(def)) return err;
    return {};
}

struct RopeConversion {
    b3::JointId rope;
    b3::JointId narrowed;
    bool converted;
    fault hinge_refusal;
    fault rope_refusal;
}

// The two conversions are taken on the same pair of bodies in the same world, so the only thing
// separating the joint each one refuses is its type.
fn RopeConversion the_conversions_of_a_rope_and_a_hinge() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -1.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 1.0);
    b3::JointId rope = link_with_a_distance_joint(world, a, b);
    b3::JointId hinge = link_with_a_revolute_joint(world, a, b);

    RopeConversion conversion = {.rope = rope};
    if (try narrowed = rope.as_distance()) {
        conversion.narrowed = narrowed;
        conversion.converted = true;
    }
    if (catch err = hinge.as_distance()) conversion.hinge_refusal = err;
    if (catch err = rope.as_revolute()) conversion.rope_refusal = err;
    return conversion;
}

struct DestroyedRopeConversion {
    bool valid_before;
    b3::JointType type_after;
    fault refusal;
}

// The joint is destroyed while its world lives on, so the handle names a joint slot box3d has
// freed rather than a world that is gone. box3d leaves the slot's type where it was, which is why
// reading the type is not on its own enough to refuse the handle.
fn DestroyedRopeConversion the_conversion_of_a_destroyed_rope() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -1.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 1.0);
    b3::JointId rope = link_with_a_distance_joint(world, a, b);

    DestroyedRopeConversion conversion = {.valid_before = rope.is_valid()};
    rope.destroy(false);
    conversion.type_after = rope.type();
    if (catch err = rope.as_distance()) conversion.refusal = err;
    return conversion;
}

fn void a_distance_joint_reports_its_own_type() @test {
    DistanceHandleReport report = the_shared_accessors_on_a_new_rope();

    assert(report.type == b3::JointType.DISTANCE);
}

fn void the_shared_joint_accessors_reach_a_distance_handle_uncast() @test {
    DistanceHandleReport report = the_shared_accessors_on_a_new_rope();

    assert(report.reported_a.index1 == report.expected_a.index1, "body_a() takes the handle as it is");
    assert(report.reported_a.world0 == report.expected_a.world0);
    assert(report.reported_a.generation == report.expected_a.generation);
    assert(report.valid, "and so do type() and is_valid()");
}

fn void a_zeroed_distance_joint_definition_is_refused() @test {
    b3::DistanceJointDef zeroed;

    fault refusal = refusal_of_a_distance_joint_definition(&zeroed, true);

    assert(refusal == b3::INVALID_DEFINITION, "box3d accepts a zeroed joint definition; the binding must not");
}

fn void a_distance_joint_definition_naming_no_bodies_is_refused() @test {
    b3::DistanceJointDef bodiless = b3::default_distance_joint_def();

    fault refusal = refusal_of_a_distance_joint_definition(&bodiless, false);

    assert(refusal == b3::INVALID_DEFINITION,
        "an unset body identifier indexes outside box3d's body array, which creating the joint writes through");

    fault none;
    b3::DistanceJointDef named = b3::default_distance_joint_def();
    fault accepted = refusal_of_a_distance_joint_definition(&named, true);
    assert(accepted == none, "the same definition naming two bodies of the calling world is accepted");
}

fn void a_distance_joint_and_a_revolute_joint_do_not_convert_to_each_other() @test {
    RopeConversion conversion = the_conversions_of_a_rope_and_a_hinge();

    assert(conversion.converted, "as_distance takes back the joint it was given");
    assert(conversion.narrowed.index1 == conversion.rope.index1);
    assert(conversion.narrowed.world0 == conversion.rope.world0);
    assert(conversion.narrowed.generation == conversion.rope.generation);
    assert(conversion.hinge_refusal == b3::WRONG_JOINT_TYPE,
        "a distance accessor on a hinge reinterprets the hinge's solver union");
    assert(conversion.rope_refusal == b3::WRONG_JOINT_TYPE,
        "and a revolute accessor on a distance joint reinterprets that joint's");
}

fn void a_destroyed_joint_does_not_narrow_to_a_distance_handle() @test {
    DestroyedRopeConversion conversion = the_conversion_of_a_destroyed_rope();

    assert(conversion.valid_before, "the joint was live before it was destroyed");
    assert(conversion.type_after == b3::JointType.DISTANCE,
        "the freed slot still reports the type it held, so the type test alone would hand the handle back");
    assert(conversion.refusal == b3::INVALID_HANDLE,
        "a handle to a joint that no longer exists is refused before its type is read");
}

fn b3::BodyId a_dynamic_sphere_below_the_origin(b3::WorldId world) {
    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, -1.0, 0.0};
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.25};
    body.create_sphere_shape(&shape_def, &sphere)!!;
    return body;
}

struct RopeLengths {
    float rest;
    float current;
    b3::Pos anchor;
    b3::Pos hanging;
}

// The body starts one length unit under the anchor and the rope is twice that, so the fall is
// the rope taking up its own slack under the world's default gravity of -10. Five seconds at
// 1/60 s is far longer than that takes, so what is left is the residual the solver holds rather
// than a body still moving. The two positions come back beside the two length readings so that
// the caller can check the accessor against the geometry and not only against itself.
fn RopeLengths the_lengths_of_a_hanging_rope() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_below_the_origin(world);

    b3::DistanceJointDef joint_def = b3::default_distance_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = hanging;
    joint_def.length = 2.0;
    b3::DistanceJointId rope = world.create_distance_joint(&joint_def)!!;

    for (int i = 0; i < 300; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return {
        .rest = rope.length(),
        .current = rope.current_length(),
        .anchor = anchor.position(),
        .hanging = hanging.position(),
    };
}

struct RopeRestLengths {
    float clamped;
    float round_tripped;
}

// Both readings come off the same joint, the clamped one first: the second is a length well
// inside the range, so it separates a setter that stores what it is given from one that always
// hands back the floor.
fn RopeRestLengths the_rest_lengths_a_rope_takes_from_its_setter() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_below_the_origin(world);
    b3::DistanceJointId rope = link_with_a_distance_joint(world, anchor, hanging);

    rope.set_length(-5.0);
    float clamped = rope.length();
    rope.set_length(3.5);
    float round_tripped = rope.length();

    return {.clamped = clamped, .round_tripped = round_tripped};
}

struct RopeConstructionLengths {
    float under_the_slop;
    float over_the_ceiling;
    float through_the_setter;
}

// The floor is common to construction and the setter, so a length past the ceiling is the only
// thing that separates them: the second rope is built at one and then handed the same number
// again through the setter, so the two readings come off one joint with nothing but the path
// between them.
fn RopeConstructionLengths the_rest_lengths_a_rope_takes_from_its_definition() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_below_the_origin(world);

    b3::DistanceJointDef joint_def = b3::default_distance_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = hanging;
    joint_def.length = 0.0;
    b3::DistanceJointId slack = world.create_distance_joint(&joint_def)!!;

    joint_def.length = 1.0e7;
    b3::DistanceJointId stretched = world.create_distance_joint(&joint_def)!!;

    RopeConstructionLengths lengths = {
        .under_the_slop = slack.length(),
        .over_the_ceiling = stretched.length(),
    };

    stretched.set_length(1.0e7);
    lengths.through_the_setter = stretched.length();
    return lengths;
}

// Construction does not run the setter's clamp: it raises the definition's length to the slop and
// gives it no ceiling of its own, so a definition keeps a length the setter would have cut down.
// A zero length is not refused either, because the check that would have refused it is compiled
// out of this build. The slop and box3d's huge value both scale with the length unit, so neither
// is asserted as a bare number; 1.0e7 is the definition's own and is exactly representable.
fn void the_rest_length_of_a_new_rope_is_the_one_construction_stored() @test {
    RopeConstructionLengths lengths = the_rest_lengths_a_rope_takes_from_its_definition();

    assert(lengths.under_the_slop == 0.005f * b3::length_units_per_meter(),
        "a definition length of zero arrives raised to the slop rather than refused");
    assert(lengths.over_the_ceiling == 1.0e7f,
        "and one far past the setter's ceiling arrives untouched, because construction has none");
    assert(lengths.through_the_setter == 1.0e5f * b3::length_units_per_meter(),
        "while the setter handed that same length cuts it down to box3d's huge value");
}

// The rope takes up the fall and holds the body at the length it was built with. The band
// excludes both the separation the pair starts at and the fall a body nothing held would have
// taken. The separation is worked out from the two body positions as well as read off the joint,
// so the accessor answers to where the bodies are and not only to itself. The last band is the
// tight one on purpose: current_length() and the geometry agree exactly, while the rest length
// stands 0.00007 away, so anything looser than that gap would let a reading of the rest length
// through — and that is the reading this one is most easily crossed with.
fn void a_distance_joint_holds_two_bodies_at_its_length() @test {
    RopeLengths lengths = the_lengths_of_a_hanging_rope();

    float dx = lengths.hanging.x - lengths.anchor.x;
    float dy = lengths.hanging.y - lengths.anchor.y;
    float dz = lengths.hanging.z - lengths.anchor.z;
    float separation = math::sqrt(dx * dx + dy * dy + dz * dz);

    assert(separation > 1.999f && separation < 2.001f,
        "the two bodies stand two length units apart, neither at the one they started at nor in free fall");
    assert(lengths.current > 1.999f && lengths.current < 2.001f, "and current_length() reports that separation");
    assert(math::abs(lengths.current - separation) < 0.00001f,
        "the joint measures its anchors, so its reading and the geometry agree");
}

// The point of the pair. The rest length is the 2 the definition carried and the measured length
// is the 2.000070 the solver settles at, so a binding that crossed the two over would report one
// number twice and fail here. 2 is exactly representable and is copied straight out of the
// definition, so it compares exactly.
fn void the_rest_length_and_the_measured_length_of_a_rope_are_two_different_readings() @test {
    RopeLengths lengths = the_lengths_of_a_hanging_rope();

    assert(lengths.rest == 2.0, "the rest length is the definition's, and the run leaves it alone");

    float stretch = lengths.current - lengths.rest;
    assert(stretch > 0.00001f && stretch < 0.001f,
        "the measured length stands a little past the rest length, by the residual the solver holds");
}

// The floor is box3d's linear slop, which is not a fixed number: it scales with the length unit,
// so the assertion is against that expression and not against the 0.005 it takes at unit scale.
fn void a_rest_length_under_box3ds_slop_is_raised_to_it() @test {
    RopeRestLengths lengths = the_rest_lengths_a_rope_takes_from_its_setter();

    assert(lengths.clamped == 0.005f * b3::length_units_per_meter(),
        "a rest length below the slop is clamped up to it rather than refused");
    assert(lengths.round_tripped == 3.5, "and a length inside the range is stored as it was given");
}

// The rope is built at twice the separation the bodies start at, so the fall takes up its slack
// under the world's default gravity of -10 and five seconds at 1/60 s is long past the settling.
// A rigid rope ends at its rest length and a sprung one hangs well below it, and the two are far
// enough apart that a single reading tells them apart. The four settings the callers pass differ
// from each other one flag at a time, which is what pins the condition down: any three of them
// on their own are consistent with a wrong reading of it.
fn float the_measured_length_of_a_hanging_rope(
  bool enable_spring,
  bool enable_limit,
  float min_length,
  float max_length,
) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_below_the_origin(world);

    b3::DistanceJointDef joint_def = b3::default_distance_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = hanging;
    joint_def.length = 2.0;
    joint_def.hertz = 1.0;
    joint_def.damping_ratio = 0.5;
    joint_def.enable_spring = enable_spring;
    joint_def.enable_limit = enable_limit;
    joint_def.min_length = min_length;
    joint_def.max_length = max_length;
    b3::DistanceJointId rope = world.create_distance_joint(&joint_def)!!;

    for (int i = 0; i < 300; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return rope.current_length();
}

struct RopeLengthRange {
    float from_construction_min;
    float from_construction_max;
    float reversed_min;
    float reversed_max;
    float under_the_slop_min;
    float under_the_slop_max;
}

// One rope read three times, the pair construction stored first so that the setter cannot have
// touched it. The reversed pair sits well inside the range box3d clamps to, so it separates the
// ordering from the clamping, and the pair under the floor is read afterwards to show that the
// second call replaces the first rather than accumulating.
fn RopeLengthRange the_length_ranges_a_rope_takes_from_its_setter() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_below_the_origin(world);
    b3::DistanceJointId rope = link_with_a_distance_joint(world, anchor, hanging);

    RopeLengthRange range = {
        .from_construction_min = rope.min_length(),
        .from_construction_max = rope.max_length(),
    };

    rope.set_length_range(3.0, 1.0);
    range.reversed_min = rope.min_length();
    range.reversed_max = rope.max_length();

    rope.set_length_range(-9.0, -9.0);
    range.under_the_slop_min = rope.min_length();
    range.under_the_slop_max = rope.max_length();
    return range;
}

struct RopeCrossedLengthRange {
    float min;
    float max;
}

// A definition that looks valid: its minimum left at the default and its maximum set under the
// slop. Construction raises the minimum to the slop but takes the maximum as the larger of the
// two raw definition lengths, so the two corrections never meet.
fn RopeCrossedLengthRange the_length_range_of_a_rope_capped_under_the_slop() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_below_the_origin(world);

    b3::DistanceJointDef joint_def = b3::default_distance_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = hanging;
    joint_def.max_length = 0.001;
    b3::DistanceJointId rope = world.create_distance_joint(&joint_def)!!;

    return {.min = rope.min_length(), .max = rope.max_length()};
}

struct RopeLimitFlags {
    bool on_a_default_rope;
    bool from_the_definition;
    bool after_enabling;
    bool after_disabling;
}

fn RopeLimitFlags the_limit_flags_of_a_bare_rope_and_a_limited_one() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_below_the_origin(world);
    b3::DistanceJointId bare = link_with_a_distance_joint(world, anchor, hanging);

    b3::DistanceJointDef joint_def = b3::default_distance_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = hanging;
    joint_def.enable_limit = true;
    joint_def.min_length = 1.0;
    joint_def.max_length = 3.0;
    b3::DistanceJointId limited = world.create_distance_joint(&joint_def)!!;

    RopeLimitFlags flags = {
        .on_a_default_rope = bare.is_limit_enabled(),
        .from_the_definition = limited.is_limit_enabled(),
    };

    bare.enable_limit(true);
    limited.enable_limit(false);
    flags.after_enabling = bare.is_limit_enabled();
    flags.after_disabling = limited.is_limit_enabled();
    return flags;
}

// The first half of the condition, with the limit on and a range the rope can move inside for
// both readings, so the spring flag is the only thing that differs between them.
fn void a_distance_joint_with_room_inside_its_limit_is_rigid_until_its_spring_is_on() @test {
    float rigid = the_measured_length_of_a_hanging_rope(
      enable_spring: false,
      enable_limit: true,
      min_length: 1.0,
      max_length: 3.0,
    );
    float sprung = the_measured_length_of_a_hanging_rope(
      enable_spring: true,
      enable_limit: true,
      min_length: 1.0,
      max_length: 3.0,
    );

    assert(rigid > 1.999f && rigid < 2.001f, "with no spring the rope holds the body at its rest length");
    assert(sprung > 2.25f && sprung < 2.26f, "and with one it stretches under the weight it carries");
}

// The second half, and the counter-intuitive one: both readings have the spring on and a range
// that pins the rope to a single length, so the limit flag is the only thing that differs — and
// it is the joint whose limit is on that comes back rigid, because min_length < max_length is
// false and the limit being on is what stops that short-circuiting.
fn void a_sprung_distance_joint_goes_rigid_when_its_limit_pins_the_range_shut() @test {
    float rigid = the_measured_length_of_a_hanging_rope(
      enable_spring: true,
      enable_limit: true,
      min_length: 2.0,
      max_length: 2.0,
    );
    float sprung = the_measured_length_of_a_hanging_rope(
      enable_spring: true,
      enable_limit: false,
      min_length: 2.0,
      max_length: 2.0,
    );

    assert(rigid > 1.999f && rigid < 2.001f, "a shut range with the limit on sends the rope down the rigid branch");
    assert(sprung > 2.25f && sprung < 2.26f, "and clearing the limit on that same range leaves it sprung");
}

fn void a_reversed_length_range_is_swapped_into_order_rather_than_refused() @test {
    RopeLengthRange range = the_length_ranges_a_rope_takes_from_its_setter();

    assert(range.reversed_min == 1.0, "box3d takes the smaller of the pair as the minimum");
    assert(range.reversed_max == 3.0, "and the larger as the maximum, whichever order they arrived in");
}

// The floor is box3d's linear slop, which is not a fixed number: it scales with the length unit,
// so the assertion is against that expression and not against the 0.005 it takes at unit scale.
fn void a_length_range_under_box3ds_slop_is_raised_to_it() @test {
    RopeLengthRange range = the_length_ranges_a_rope_takes_from_its_setter();

    assert(range.under_the_slop_min == 0.005f * b3::length_units_per_meter(),
        "both ends of a range below the slop are clamped up to it rather than refused");
    assert(range.under_the_slop_max == 0.005f * b3::length_units_per_meter(),
        "and the swap then leaves the pair equal rather than inverted");
}

// Construction does not run the setter's clamp: it raises the definition's minimum to the slop
// and takes the maximum as the larger of the two it was handed, with no ceiling of its own. Both
// bounds scale with the length unit, so neither is asserted as a bare number.
fn void the_length_range_of_a_new_rope_is_the_one_construction_stored() @test {
    RopeLengthRange range = the_length_ranges_a_rope_takes_from_its_setter();

    assert(range.from_construction_min == 0.005f * b3::length_units_per_meter(),
        "a default definition's zero minimum arrives raised to the slop");
    assert(range.from_construction_max == 1.0e5f * b3::length_units_per_meter(),
        "and its huge maximum arrives untouched");
}

// The consequence of construction's two mismatched corrections, and it is reachable from a
// definition nothing about it looks wrong: the range comes back with its low end above its high
// one, which set_length_range cannot produce because it clamps both ends and then orders them.
fn void a_definition_can_leave_a_rope_whose_minimum_length_is_above_its_maximum() @test {
    RopeCrossedLengthRange range = the_length_range_of_a_rope_capped_under_the_slop();

    assert(range.min == 0.005f * b3::length_units_per_meter(),
        "the definition's default minimum arrives raised to the slop");
    assert(range.max == 0.001f, "while its maximum arrives as it was given, under that floor");
    assert(range.min > range.max, "so the two corrections leave the range crossed");
}

fn void the_limit_flag_of_a_rope_starts_off_and_round_trips() @test {
    RopeLimitFlags flags = the_limit_flags_of_a_bare_rope_and_a_limited_one();

    assert(!flags.on_a_default_rope, "a default distance joint carries no limit");
    assert(flags.from_the_definition, "the joint reports the limit its definition asked for");
    assert(flags.after_enabling);
    assert(!flags.after_disabling);
}

// The joint is built sprung and soft and then handed the stiffness under test, so the setter is
// what separates the two callers rather than the definition. Everything else matches the rigid
// and sprung fixtures above: the body starts one length unit under the anchor, the rope is twice
// that, and five seconds at 1/60 s is long past the settling.
fn float the_measured_length_of_a_sprung_rope(float hertz) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_below_the_origin(world);

    b3::DistanceJointDef joint_def = b3::default_distance_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = hanging;
    joint_def.length = 2.0;
    joint_def.enable_spring = true;
    joint_def.hertz = 1.0;
    joint_def.damping_ratio = 0.5;
    b3::DistanceJointId rope = world.create_distance_joint(&joint_def)!!;
    rope.set_spring_hertz(hertz);

    for (int i = 0; i < 300; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return rope.current_length();
}

struct RopeSpringSettings {
    bool enabled;
    float hertz;
    float damping_ratio;
    bool enabled_after;
    float hertz_after;
    float damping_ratio_after;
}

fn RopeSpringSettings the_spring_settings_of_a_rope_before_and_after_the_setters() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_below_the_origin(world);

    b3::DistanceJointDef joint_def = b3::default_distance_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = hanging;
    joint_def.enable_spring = true;
    joint_def.hertz = 2.0;
    joint_def.damping_ratio = 1.0;
    b3::DistanceJointId rope = world.create_distance_joint(&joint_def)!!;

    RopeSpringSettings settings = {
        .enabled = rope.is_spring_enabled(),
        .hertz = rope.spring_hertz(),
        .damping_ratio = rope.spring_damping_ratio(),
    };

    rope.enable_spring(false);
    rope.set_spring_hertz(8.0);
    rope.set_spring_damping_ratio(0.25);

    settings.enabled_after = rope.is_spring_enabled();
    settings.hertz_after = rope.spring_hertz();
    settings.damping_ratio_after = rope.spring_damping_ratio();
    return settings;
}

struct RopeSpringForces {
    b3::SpringForceRange from_construction;
    b3::SpringForceRange inverted;
}

// One rope read twice, the pair construction stored first so that the setter cannot have touched
// it. The inverted pair holds two different values of opposite sign, so a reading that reported
// the upper force as the lower would fail rather than agree with itself.
fn RopeSpringForces the_spring_force_ranges_a_rope_takes_from_its_setter() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_below_the_origin(world);
    b3::DistanceJointId rope = link_with_a_distance_joint(world, anchor, hanging);

    RopeSpringForces forces = {.from_construction = rope.spring_force_range()};

    rope.set_spring_force_range(7.0, -3.0);
    forces.inverted = rope.spring_force_range();
    return forces;
}

// A stiffer spring settles the rope shorter. Both ropes are sprung, carry the same damping and
// hang the same weight, so the stiffness the setter gave them is the only thing between the
// measured 2.254364 and 2.015831. The two bands are two-sided and disjoint, so neither reading
// can pass for the other.
fn void a_stiffer_spring_holds_the_rope_closer_to_its_rest_length() @test {
    float soft = the_measured_length_of_a_sprung_rope(1.0);
    float stiff = the_measured_length_of_a_sprung_rope(4.0);

    assert(soft > 2.25f && soft < 2.26f, "a soft spring lets the weight stretch the rope well past its rest length");
    assert(stiff > 2.01f && stiff < 2.02f, "and four times the stiffness holds it most of the way back");
}

// 2, 1, 8 and 0.25 are all exactly representable as floats, so they compare exactly.
fn void the_spring_settings_of_a_rope_round_trip() @test {
    RopeSpringSettings settings = the_spring_settings_of_a_rope_before_and_after_the_setters();

    assert(settings.enabled, "the joint reports the spring its definition asked for");
    assert(settings.hertz == 2.0);
    assert(settings.damping_ratio == 1.0);
    assert(!settings.enabled_after, "and then the values the setters gave it");
    assert(settings.hertz_after == 8.0);
    assert(settings.damping_ratio_after == 0.25);
}

fn void the_spring_force_range_of_a_new_rope_is_unbounded_either_way() @test {
    RopeSpringForces forces = the_spring_force_ranges_a_rope_takes_from_its_setter();

    assert(forces.from_construction.lower == -float::max, "a default definition holds the spring back by nothing");
    assert(forces.from_construction.upper == float::max, "and lets it pull by as much");
}

// The asymmetry between the joint's three range setters, pinned. set_length_range one accessor
// away clamps both ends and swaps them into order; this one does neither, because the assertion
// it would have refused an inverted pair with is compiled out of this build.
fn void an_inverted_spring_force_range_comes_back_the_way_it_arrived() @test {
    RopeSpringForces forces = the_spring_force_ranges_a_rope_takes_from_its_setter();

    assert(forces.inverted.lower == 7.0, "the lower force is the 7 it was handed, not the smaller of the pair");
    assert(forces.inverted.upper == -3.0, "and the upper force the -3, neither clamped nor swapped into order");
}

// A sphere at the rest length the motor tests build their joint with, so a joint that holds its
// length leaves the body exactly where it started and only the motor can move it.
fn b3::BodyId a_dynamic_sphere_two_units_below_the_origin(b3::WorldId world) {
    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, -2.0, 0.0};
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.25};
    body.create_sphere_shape(&shape_def, &sphere)!!;
    return body;
}

struct RopeDrive {
    float motor_force;
    b3::Vec3 constraint_force;
    float current_length;
}

// The body starts exactly the rope's rest length under a static anchor with gravity off, so the
// motor is the only thing that can move it and a measured length still at 2.0 means it did not.
// Two seconds at 1/60 s is far past the point the motor reaches whatever force it settles at.
fn RopeDrive a_driven_rope(
  bool enable_spring,
  float hertz,
  bool enable_motor,
  float max_motor_force,
) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_two_units_below_the_origin(world);

    b3::DistanceJointDef joint_def = b3::default_distance_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = hanging;
    joint_def.length = 2.0;
    joint_def.enable_spring = enable_spring;
    joint_def.hertz = hertz;
    joint_def.damping_ratio = 1.0;
    joint_def.enable_motor = enable_motor;
    joint_def.motor_speed = 0.5;
    joint_def.max_motor_force = max_motor_force;
    b3::DistanceJointId rope = world.create_distance_joint(&joint_def)!!;

    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return {
        .motor_force = rope.motor_force(),
        .constraint_force = rope.constraint_force(),
        .current_length = rope.current_length(),
    };
}

struct RopeMotorForces {
    float driven;
    float after_the_spring_flag;
    float after_the_limit_flag;
    float after_the_motor_flag;
}

// One saturated motor read four times, each reading after a flag has been toggled to the other
// value and back, and no step anywhere between them — so the flag is the only thing that can move
// the number. The spring and limit flags come first: they leave the accumulated impulse where it
// stands, which is what gives the motor flag's zero its meaning.
fn RopeMotorForces the_motor_force_of_a_driven_rope_around_toggling_its_flags() {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_two_units_below_the_origin(world);

    b3::DistanceJointDef joint_def = b3::default_distance_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = hanging;
    joint_def.length = 2.0;
    joint_def.enable_spring = true;
    joint_def.hertz = 1.0;
    joint_def.damping_ratio = 1.0;
    joint_def.enable_motor = true;
    joint_def.motor_speed = 0.5;
    joint_def.max_motor_force = 100.0;
    b3::DistanceJointId rope = world.create_distance_joint(&joint_def)!!;

    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    RopeMotorForces forces = {.driven = rope.motor_force()};

    rope.enable_spring(false);
    rope.enable_spring(true);
    forces.after_the_spring_flag = rope.motor_force();

    rope.enable_limit(true);
    rope.enable_limit(false);
    forces.after_the_limit_flag = rope.motor_force();

    rope.enable_motor(false);
    rope.enable_motor(true);
    forces.after_the_motor_flag = rope.motor_force();
    return forces;
}

struct RopeMotorSettings {
    bool enabled;
    float speed;
    float max_force;
    bool enabled_after;
    float speed_after;
    float max_force_after;
}

fn RopeMotorSettings the_motor_settings_of_a_rope_before_and_after_the_setters() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId hanging = a_dynamic_sphere_below_the_origin(world);

    b3::DistanceJointDef joint_def = b3::default_distance_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = hanging;
    joint_def.enable_motor = true;
    joint_def.motor_speed = 0.5;
    joint_def.max_motor_force = 100.0;
    b3::DistanceJointId rope = world.create_distance_joint(&joint_def)!!;

    RopeMotorSettings settings = {
        .enabled = rope.is_motor_enabled(),
        .speed = rope.motor_speed(),
        .max_force = rope.max_motor_force(),
    };

    rope.enable_motor(false);
    rope.set_motor_speed(-0.25);
    rope.set_max_motor_force(12.5);

    settings.enabled_after = rope.is_motor_enabled();
    settings.speed_after = rope.motor_speed();
    settings.max_force_after = rope.max_motor_force();
    return settings;
}

// The motor asks for 0.5 m/s against a spring holding the rope on its rest length, which is far
// more than 100 newtons buys, so the reading is the ceiling rather than a load the motor balances.
// The length band is disjoint from the 2.0 both controls below sit at.
fn void a_motor_pulling_against_the_spring_saturates_at_its_ceiling() @test {
    RopeDrive driven = a_driven_rope(
      enable_spring: true,
      hertz: 1.0,
      enable_motor: true,
      max_motor_force: 100.0,
    );

    assert(driven.motor_force > 99.9f && driven.motor_force < 100.1f,
        "the motor is held at the ceiling it was given");
    assert(driven.current_length > 2.03f && driven.current_length < 2.05f,
        "and it has driven the rope out past the rest length the spring holds it on");
}

// The first control: the same definition with only the motor flag cleared, so the speed and the
// ceiling the joint still carries drive nothing.
fn void a_rope_whose_motor_is_disabled_applies_no_motor_force() @test {
    RopeDrive idle = a_driven_rope(
      enable_spring: true,
      hertz: 1.0,
      enable_motor: false,
      max_motor_force: 100.0,
    );

    assert(idle.motor_force > -0.001f && idle.motor_force < 0.001f, "a disabled motor applies nothing");
    assert(idle.current_length > 1.999f && idle.current_length < 2.001f, "and the rope stays where it started");
}

// The second control, and the one the compound condition earns: the same motor on a joint whose
// spring is off. box3d solves that joint as a single rigid constraint and never reaches the motor,
// so both readings match the disabled motor above rather than the driven rope.
fn void a_motor_on_a_rope_whose_spring_is_off_drives_nothing() @test {
    RopeDrive rigid = a_driven_rope(
      enable_spring: false,
      hertz: 1.0,
      enable_motor: true,
      max_motor_force: 100.0,
    );

    assert(rigid.motor_force > -0.001f && rigid.motor_force < 0.001f,
        "the rigid branch never reaches the motor, whatever it was set to");
    assert(rigid.current_length > 1.999f && rigid.current_length < 2.001f, "so the rope holds its rest length");
}

// Same accessor name, different contract, within one joint type: the motor flag discards the
// impulse it had accumulated and the spring and limit flags beside it discard nothing. The two
// controls are what make this discriminate — they are the same shape of call on the same joint,
// so a reading that survives them and then falls to zero can only have been the motor flag.
fn void toggling_the_motor_discards_the_force_it_had_where_the_other_two_flags_do_not() @test {
    RopeMotorForces forces = the_motor_force_of_a_driven_rope_around_toggling_its_flags();

    assert(forces.driven > 99.9f && forces.driven < 100.1f, "the motor is held at the ceiling it was given");
    assert(forces.after_the_spring_flag > 99.9f && forces.after_the_spring_flag < 100.1f,
        "toggling the spring leaves the motor's accumulated force alone");
    assert(forces.after_the_limit_flag > 99.9f && forces.after_the_limit_flag < 100.1f,
        "and so does toggling the limit");
    assert(forces.after_the_motor_flag > -0.001f && forces.after_the_motor_flag < 0.001f,
        "while toggling the motor throws that force away, before any step could rebuild it");
}

// 0.5, 100, -0.25 and 12.5 are all exactly representable as floats, so they compare exactly.
fn void the_motor_settings_of_a_rope_round_trip() @test {
    RopeMotorSettings settings = the_motor_settings_of_a_rope_before_and_after_the_setters();

    assert(settings.enabled, "the joint reports the motor its definition asked for");
    assert(settings.speed == 0.5);
    assert(settings.max_force == 100.0);
    assert(!settings.enabled_after, "and then the values the setters gave it");
    assert(settings.speed_after == -0.25);
    assert(settings.max_force_after == 12.5);
}

// The relationship a consumer reaching for the shared accessor needs, and it is not the revolute
// joint's: box3d sums this joint's axial impulses once and lays the total along the segment
// between the anchors, so with the spring silent at 0 hertz and no limit the motor is the whole
// sum. The vector points from the anchor down to the hanging body, which is the direction the
// motor drives it, so a positive motor force reads as a negative y component.
fn void the_shared_constraint_force_carries_a_distance_motor_force_once() @test {
    RopeDrive driven = a_driven_rope(
      enable_spring: true,
      hertz: 0.0,
      enable_motor: true,
      max_motor_force: 10.0,
    );

    assert(driven.motor_force > 9.99f && driven.motor_force < 10.01f, "the motor is held at the ceiling it was given");
    assert(driven.constraint_force.x > -0.001f && driven.constraint_force.x < 0.001f,
        "the whole force lies along the segment between the anchors");
    assert(driven.constraint_force.z > -0.001f && driven.constraint_force.z < 0.001f);

    float ratio = -driven.constraint_force.y / driven.motor_force;
    assert(ratio > 0.999f && ratio < 1.001f, "the shared accessor carries the motor force once, not twice");
}

// The other half of that relationship: once the spring is working the two readings are not a
// multiple of each other at all. The spring pulls back against the motor and the shared accessor
// reports the net, so a motor saturated at 100 newtons leaves it under 2.
fn void a_spring_pulling_back_leaves_the_shared_force_far_under_the_motor_force() @test {
    RopeDrive driven = a_driven_rope(
      enable_spring: true,
      hertz: 1.0,
      enable_motor: true,
      max_motor_force: 100.0,
    );

    assert(driven.motor_force > 99.9f && driven.motor_force < 100.1f, "the motor is held at its ceiling");
    assert(driven.constraint_force.y > 1.8f && driven.constraint_force.y < 1.95f,
        "while the joint's whole force is the small remainder the spring does not cancel");
}

fn void the_prismatic_joint_types_match_the_c_layout() @test {
    assert(b3::PrismaticJointDef::size == 152);
    assert(b3::PrismaticJointDef::alignment == 8);
}

fn void the_weld_joint_types_match_the_c_layout() @test {
    assert(b3::WeldJointDef::size == 128);
    assert(b3::WeldJointDef::alignment == 8);
}

fn void the_parallel_joint_types_match_the_c_layout() @test {
    assert(b3::ParallelJointDef::size == 128);
    assert(b3::ParallelJointDef::alignment == 8);
}

// 0 is exactly representable as a float and is copied straight out of the definition, so these
// compare exactly rather than in a band.
fn void the_default_prismatic_joint_definition_zeroes_every_field_it_adds_to_the_base() @test {
    b3::PrismaticJointDef def = b3::default_prismatic_joint_def();

    assert(!def.enable_spring, "a default prismatic joint is a bare slider");
    assert(def.hertz == 0.0);
    assert(def.damping_ratio == 0.0);
    assert(def.target_translation == 0.0);
    assert(!def.enable_limit);
    assert(def.lower_translation == 0.0);
    assert(def.upper_translation == 0.0);
    assert(!def.enable_motor);
    assert(def.max_motor_force == 0.0, "0, not the float::max the base thresholds carry");
    assert(def.motor_speed == 0.0);
}

fn void a_default_prismatic_joint_definition_carries_the_cookie_every_definition_shares() @test {
    b3::PrismaticJointDef def = b3::default_prismatic_joint_def();
    b3::FilterJointDef filter_def = b3::default_filter_joint_def();

    assert(def.internal_value == def.base.internal_value, "the direct field names the embedded base");
    assert(def.internal_value == filter_def.internal_value, "every definition starts from the same base");
}

// Zero is what this type asks for a rigid channel with, so a default weld joint is the stiff one
// rather than the free one the same zeroes mean on the other sprung types.
fn void the_default_weld_joint_definition_zeroes_all_four_fields_it_adds_to_the_base() @test {
    b3::WeldJointDef def = b3::default_weld_joint_def();

    assert(def.linear_hertz == 0.0);
    assert(def.angular_hertz == 0.0);
    assert(def.linear_damping_ratio == 0.0);
    assert(def.angular_damping_ratio == 0.0);
}

fn void a_default_weld_joint_definition_carries_the_cookie_every_definition_shares() @test {
    b3::WeldJointDef def = b3::default_weld_joint_def();
    b3::FilterJointDef filter_def = b3::default_filter_joint_def();

    assert(def.internal_value == def.base.internal_value, "the direct field names the embedded base");
    assert(def.internal_value == filter_def.internal_value, "every definition starts from the same base");
}

// The odd one of the three: 1 and float::max are exactly representable as floats and are copied
// straight out of the definition, so they too compare exactly.
fn void the_default_parallel_joint_definition_zeroes_nothing_it_adds_to_the_base() @test {
    b3::ParallelJointDef def = b3::default_parallel_joint_def();

    assert(def.hertz == 1.0, "a default parallel joint is already sprung");
    assert(def.damping_ratio == 1.0);
    assert(def.max_torque == float::max, "and its spring is uncapped, where 0 would switch it off");
}

fn void a_default_parallel_joint_definition_carries_the_cookie_every_definition_shares() @test {
    b3::ParallelJointDef def = b3::default_parallel_joint_def();
    b3::FilterJointDef filter_def = b3::default_filter_joint_def();

    assert(def.internal_value == def.base.internal_value, "the direct field names the embedded base");
    assert(def.internal_value == filter_def.internal_value, "every definition starts from the same base");
}

fn b3::PrismaticJointId link_with_a_prismatic_joint(b3::WorldId world, b3::BodyId a, b3::BodyId b) {
    b3::PrismaticJointDef joint_def = b3::default_prismatic_joint_def();
    joint_def.body_id_a = a;
    joint_def.body_id_b = b;
    return world.create_prismatic_joint(&joint_def)!!;
}

fn b3::WeldJointId link_with_a_weld_joint(b3::WorldId world, b3::BodyId a, b3::BodyId b) {
    b3::WeldJointDef joint_def = b3::default_weld_joint_def();
    joint_def.body_id_a = a;
    joint_def.body_id_b = b;
    return world.create_weld_joint(&joint_def)!!;
}

fn b3::ParallelJointId link_with_a_parallel_joint(b3::WorldId world, b3::BodyId a, b3::BodyId b) {
    b3::ParallelJointDef joint_def = b3::default_parallel_joint_def();
    joint_def.body_id_a = a;
    joint_def.body_id_b = b;
    return world.create_parallel_joint(&joint_def)!!;
}

struct TypedHandleReport {
    b3::JointType type;
    b3::BodyId expected_a;
    b3::BodyId reported_a;
    bool valid;
}

// Every accessor here is a shared JointId one called on the typed handle with no cast, which is
// what the inline typedef buys.
fn TypedHandleReport the_shared_accessors_on_a_new_slider() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -1.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 1.0);
    b3::PrismaticJointId slider = link_with_a_prismatic_joint(world, a, b);

    return {
        .type = slider.type(),
        .expected_a = a,
        .reported_a = slider.body_a(),
        .valid = slider.is_valid(),
    };
}

fn TypedHandleReport the_shared_accessors_on_a_new_weld_joint() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -1.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 1.0);
    b3::WeldJointId weld = link_with_a_weld_joint(world, a, b);

    return {
        .type = weld.type(),
        .expected_a = a,
        .reported_a = weld.body_a(),
        .valid = weld.is_valid(),
    };
}

fn TypedHandleReport the_shared_accessors_on_a_new_parallel_joint() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -1.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 1.0);
    b3::ParallelJointId parallel = link_with_a_parallel_joint(world, a, b);

    return {
        .type = parallel.type(),
        .expected_a = a,
        .reported_a = parallel.body_a(),
        .valid = parallel.is_valid(),
    };
}

fn void the_shared_joint_accessors_reach_a_prismatic_handle_uncast() @test {
    TypedHandleReport report = the_shared_accessors_on_a_new_slider();

    assert(report.type == b3::JointType.PRISMATIC);
    assert(report.reported_a.index1 == report.expected_a.index1, "body_a() takes the handle as it is");
    assert(report.reported_a.world0 == report.expected_a.world0);
    assert(report.reported_a.generation == report.expected_a.generation);
    assert(report.valid, "and so do type() and is_valid()");
}

fn void the_shared_joint_accessors_reach_a_weld_handle_uncast() @test {
    TypedHandleReport report = the_shared_accessors_on_a_new_weld_joint();

    assert(report.type == b3::JointType.WELD);
    assert(report.reported_a.index1 == report.expected_a.index1, "body_a() takes the handle as it is");
    assert(report.reported_a.world0 == report.expected_a.world0);
    assert(report.reported_a.generation == report.expected_a.generation);
    assert(report.valid, "and so do type() and is_valid()");
}

fn void the_shared_joint_accessors_reach_a_parallel_handle_uncast() @test {
    TypedHandleReport report = the_shared_accessors_on_a_new_parallel_joint();

    assert(report.type == b3::JointType.PARALLEL);
    assert(report.reported_a.index1 == report.expected_a.index1, "body_a() takes the handle as it is");
    assert(report.reported_a.world0 == report.expected_a.world0);
    assert(report.reported_a.generation == report.expected_a.generation);
    assert(report.valid, "and so do type() and is_valid()");
}

struct TypedJointConversions {
    b3::JointId slider;
    b3::JointId weld;
    b3::JointId parallel;
    b3::JointId narrowed_slider;
    b3::JointId narrowed_weld;
    b3::JointId narrowed_parallel;
    bool slider_converted;
    bool weld_converted;
    bool parallel_converted;
    fault weld_as_slider;
    fault parallel_as_slider;
    fault slider_as_weld;
    fault parallel_as_weld;
    fault slider_as_parallel;
    fault weld_as_parallel;
}

// All three joints link the same pair of bodies in the same world, so the only thing separating
// the joint each conversion refuses is its type.
fn TypedJointConversions the_conversions_of_a_slider_a_weld_joint_and_a_parallel_joint() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -1.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 1.0);
    b3::JointId slider = link_with_a_prismatic_joint(world, a, b);
    b3::JointId weld = link_with_a_weld_joint(world, a, b);
    b3::JointId parallel = link_with_a_parallel_joint(world, a, b);

    TypedJointConversions conversions = {.slider = slider, .weld = weld, .parallel = parallel};
    if (try narrowed = slider.as_prismatic()) {
        conversions.narrowed_slider = narrowed;
        conversions.slider_converted = true;
    }
    if (try narrowed = weld.as_weld()) {
        conversions.narrowed_weld = narrowed;
        conversions.weld_converted = true;
    }
    if (try narrowed = parallel.as_parallel()) {
        conversions.narrowed_parallel = narrowed;
        conversions.parallel_converted = true;
    }
    if (catch err = weld.as_prismatic()) conversions.weld_as_slider = err;
    if (catch err = parallel.as_prismatic()) conversions.parallel_as_slider = err;
    if (catch err = slider.as_weld()) conversions.slider_as_weld = err;
    if (catch err = parallel.as_weld()) conversions.parallel_as_weld = err;
    if (catch err = slider.as_parallel()) conversions.slider_as_parallel = err;
    if (catch err = weld.as_parallel()) conversions.weld_as_parallel = err;
    return conversions;
}

struct DestroyedTypedConversions {
    bool valid_before;
    b3::JointType slider_type_after;
    fault slider_refusal;
    fault weld_refusal;
    fault parallel_refusal;
}

// The three joints are destroyed while their world lives on, so each handle names a joint slot
// box3d has freed rather than a world that is gone. box3d leaves the slot's type where it was,
// which is why reading the type is not on its own enough to refuse the handle.
fn DestroyedTypedConversions the_conversions_of_three_destroyed_joints() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -1.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 1.0);
    b3::JointId slider = link_with_a_prismatic_joint(world, a, b);
    b3::JointId weld = link_with_a_weld_joint(world, a, b);
    b3::JointId parallel = link_with_a_parallel_joint(world, a, b);

    DestroyedTypedConversions conversions = {
        .valid_before = slider.is_valid() && weld.is_valid() && parallel.is_valid(),
    };
    slider.destroy(false);
    weld.destroy(false);
    parallel.destroy(false);

    conversions.slider_type_after = slider.type();
    if (catch err = slider.as_prismatic()) conversions.slider_refusal = err;
    if (catch err = weld.as_weld()) conversions.weld_refusal = err;
    if (catch err = parallel.as_parallel()) conversions.parallel_refusal = err;
    return conversions;
}

fn fault refusal_of_a_prismatic_joint_definition(b3::PrismaticJointDef* def, bool name_bodies) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    if (name_bodies) {
        def.body_id_a = a_dynamic_sphere_at(world, -1.0);
        def.body_id_b = a_dynamic_sphere_at(world, 1.0);
    }

    if (catch err = world.create_prismatic_joint(def)) return err;
    return {};
}

fn fault refusal_of_a_weld_joint_definition(b3::WeldJointDef* def, bool name_bodies) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    if (name_bodies) {
        def.body_id_a = a_dynamic_sphere_at(world, -1.0);
        def.body_id_b = a_dynamic_sphere_at(world, 1.0);
    }

    if (catch err = world.create_weld_joint(def)) return err;
    return {};
}

fn fault refusal_of_a_parallel_joint_definition(b3::ParallelJointDef* def, bool name_bodies) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    if (name_bodies) {
        def.body_id_a = a_dynamic_sphere_at(world, -1.0);
        def.body_id_b = a_dynamic_sphere_at(world, 1.0);
    }

    if (catch err = world.create_parallel_joint(def)) return err;
    return {};
}

fn void only_a_prismatic_joint_narrows_to_a_prismatic_handle() @test {
    TypedJointConversions conversions = the_conversions_of_a_slider_a_weld_joint_and_a_parallel_joint();

    assert(conversions.slider_converted, "as_prismatic takes back the joint it was given");
    assert(conversions.narrowed_slider.index1 == conversions.slider.index1);
    assert(conversions.narrowed_slider.world0 == conversions.slider.world0);
    assert(conversions.narrowed_slider.generation == conversions.slider.generation);
    assert(conversions.weld_as_slider == b3::WRONG_JOINT_TYPE,
        "a prismatic accessor on a weld joint reinterprets the weld joint's solver union");
    assert(conversions.parallel_as_slider == b3::WRONG_JOINT_TYPE, "and on a parallel joint, that joint's");
}

fn void only_a_weld_joint_narrows_to_a_weld_handle() @test {
    TypedJointConversions conversions = the_conversions_of_a_slider_a_weld_joint_and_a_parallel_joint();

    assert(conversions.weld_converted, "as_weld takes back the joint it was given");
    assert(conversions.narrowed_weld.index1 == conversions.weld.index1);
    assert(conversions.narrowed_weld.world0 == conversions.weld.world0);
    assert(conversions.narrowed_weld.generation == conversions.weld.generation);
    assert(conversions.slider_as_weld == b3::WRONG_JOINT_TYPE,
        "a weld accessor on a slider reinterprets the slider's solver union");
    assert(conversions.parallel_as_weld == b3::WRONG_JOINT_TYPE, "and on a parallel joint, that joint's");
}

fn void only_a_parallel_joint_narrows_to_a_parallel_handle() @test {
    TypedJointConversions conversions = the_conversions_of_a_slider_a_weld_joint_and_a_parallel_joint();

    assert(conversions.parallel_converted, "as_parallel takes back the joint it was given");
    assert(conversions.narrowed_parallel.index1 == conversions.parallel.index1);
    assert(conversions.narrowed_parallel.world0 == conversions.parallel.world0);
    assert(conversions.narrowed_parallel.generation == conversions.parallel.generation);
    assert(conversions.slider_as_parallel == b3::WRONG_JOINT_TYPE,
        "a parallel accessor on a slider reinterprets the slider's solver union");
    assert(conversions.weld_as_parallel == b3::WRONG_JOINT_TYPE, "and on a weld joint, that joint's");
}

fn void a_destroyed_joint_narrows_to_none_of_the_three_handles() @test {
    DestroyedTypedConversions conversions = the_conversions_of_three_destroyed_joints();

    assert(conversions.valid_before, "all three joints were live before they were destroyed");
    assert(conversions.slider_type_after == b3::JointType.PRISMATIC,
        "the freed slot still reports the type it held, so the type test alone would hand the handle back");
    assert(conversions.slider_refusal == b3::INVALID_HANDLE,
        "a handle to a joint that no longer exists is refused before its type is read");
    assert(conversions.weld_refusal == b3::INVALID_HANDLE);
    assert(conversions.parallel_refusal == b3::INVALID_HANDLE);
}

fn void a_zeroed_definition_of_each_new_joint_type_is_refused() @test {
    b3::PrismaticJointDef zeroed_slider;
    b3::WeldJointDef zeroed_weld;
    b3::ParallelJointDef zeroed_parallel;

    assert(refusal_of_a_prismatic_joint_definition(&zeroed_slider, true) == b3::INVALID_DEFINITION,
        "box3d accepts a zeroed joint definition; the binding must not");
    assert(refusal_of_a_weld_joint_definition(&zeroed_weld, true) == b3::INVALID_DEFINITION);
    assert(refusal_of_a_parallel_joint_definition(&zeroed_parallel, true) == b3::INVALID_DEFINITION);
}

fn void a_definition_of_each_new_joint_type_naming_no_bodies_is_refused() @test {
    b3::PrismaticJointDef bodiless_slider = b3::default_prismatic_joint_def();
    b3::WeldJointDef bodiless_weld = b3::default_weld_joint_def();
    b3::ParallelJointDef bodiless_parallel = b3::default_parallel_joint_def();

    assert(refusal_of_a_prismatic_joint_definition(&bodiless_slider, false) == b3::INVALID_DEFINITION,
        "an unset body identifier indexes outside box3d's body array, which creating the joint writes through");
    assert(refusal_of_a_weld_joint_definition(&bodiless_weld, false) == b3::INVALID_DEFINITION);
    assert(refusal_of_a_parallel_joint_definition(&bodiless_parallel, false) == b3::INVALID_DEFINITION);

    fault none;
    b3::PrismaticJointDef named_slider = b3::default_prismatic_joint_def();
    b3::WeldJointDef named_weld = b3::default_weld_joint_def();
    b3::ParallelJointDef named_parallel = b3::default_parallel_joint_def();

    assert(refusal_of_a_prismatic_joint_definition(&named_slider, true) == none,
        "the same definition naming two bodies of the calling world is accepted");
    assert(refusal_of_a_weld_joint_definition(&named_weld, true) == none);
    assert(refusal_of_a_parallel_joint_definition(&named_parallel, true) == none);
}

fn b3::BodyId a_small_dynamic_sphere_at_the_origin(b3::WorldId world) {
    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.25};
    body.create_sphere_shape(&shape_def, &sphere)!!;
    return body;
}

struct SliderDrive {
    float translation;
    float speed;
}

// One second at 1/60 s against a static anchor, under a gravity the joint has to hold off rather
// than one that drives anything: the slider's axis is the one direction the body can move. The
// motor reaches the half-metre limit halfway through the run and is still sitting on it at the
// end — doubling the run leaves the limited reading at the same 0.500108 — so the limited band
// holds the limit rather than a run that stopped early.
fn SliderDrive a_motor_driven_slider_after_a_second(bool enable_limit) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, -10.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId sliding = a_small_dynamic_sphere_at_the_origin(world);

    b3::PrismaticJointDef joint_def = b3::default_prismatic_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = sliding;
    joint_def.enable_motor = true;
    joint_def.motor_speed = 1.0;
    joint_def.max_motor_force = 1000.0;
    joint_def.enable_limit = enable_limit;
    joint_def.lower_translation = -0.5;
    joint_def.upper_translation = 0.5;
    b3::PrismaticJointId slider = world.create_prismatic_joint(&joint_def)!!;

    for (int i = 0; i < 60; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return {.translation = slider.translation(), .speed = slider.speed()};
}

struct SliderLimits {
    float lower;
    float upper;
}

fn SliderLimits the_limits_of_a_slider_set_to(float lower, float upper) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId sliding = a_small_dynamic_sphere_at_the_origin(world);
    b3::PrismaticJointId slider = link_with_a_prismatic_joint(world, anchor, sliding);

    slider.set_limits(lower, upper);
    return {.lower = slider.lower_limit(), .upper = slider.upper_limit()};
}

fn SliderLimits the_limits_of_a_slider_whose_definition_reversed_them() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId sliding = a_small_dynamic_sphere_at_the_origin(world);

    b3::PrismaticJointDef joint_def = b3::default_prismatic_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = sliding;
    joint_def.enable_limit = true;
    joint_def.lower_translation = 1.0;
    joint_def.upper_translation = -1.0;
    b3::PrismaticJointId slider = world.create_prismatic_joint(&joint_def)!!;

    return {.lower = slider.lower_limit(), .upper = slider.upper_limit()};
}

struct SliderLimitFlags {
    bool on_a_default_slider;
    bool from_the_definition;
    bool after_enabling;
    bool after_disabling;
}

fn SliderLimitFlags the_limit_flags_of_a_bare_slider_and_a_limited_one() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId sliding = a_small_dynamic_sphere_at_the_origin(world);
    b3::PrismaticJointId bare = link_with_a_prismatic_joint(world, anchor, sliding);

    b3::PrismaticJointDef joint_def = b3::default_prismatic_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = sliding;
    joint_def.enable_limit = true;
    joint_def.lower_translation = -0.5;
    joint_def.upper_translation = 0.5;
    b3::PrismaticJointId limited = world.create_prismatic_joint(&joint_def)!!;

    SliderLimitFlags flags = {
        .on_a_default_slider = bare.is_limit_enabled(),
        .from_the_definition = limited.is_limit_enabled(),
    };

    bare.enable_limit(true);
    limited.enable_limit(false);
    flags.after_enabling = bare.is_limit_enabled();
    flags.after_disabling = limited.is_limit_enabled();
    return flags;
}

// 0.969331 rather than the nominal 1.0 because the solver ramps the motor up over the first
// steps. The band holds that and excludes both a slider that never moved and the 0.500108 the
// same definition reaches once its limit is armed.
fn void a_motor_slides_the_body_along_the_joints_axis() @test {
    SliderDrive driven = a_motor_driven_slider_after_a_second(false);

    assert(driven.translation > 0.94 && driven.translation < 0.99,
        "a motor at 1 m/s carries the slider nearly a metre in a second");
}

// The control for the one above: the same definition with only the limit flag set. The two bands
// are disjoint, so neither reading can be mistaken for the other.
fn void a_limit_stops_the_motor_that_drives_the_slider_into_it() @test {
    SliderDrive held = a_motor_driven_slider_after_a_second(true);

    assert(held.translation > 0.49 && held.translation < 0.51,
        "the motor is held at the half-metre limit rather than sliding past it");
}

// The limited row is what makes this reading discriminating: the motor speed the joint was given
// still reads 1 there, and speed() is the slider standing still.
fn void the_slider_moves_at_the_speed_its_motor_asks_for_until_the_limit_takes_it() @test {
    SliderDrive driven = a_motor_driven_slider_after_a_second(false);
    SliderDrive held = a_motor_driven_slider_after_a_second(true);

    assert(driven.speed > 0.95 && driven.speed < 1.05, "the motor has the slider at the metre a second it asked for");
    assert(held.speed > -0.01 && held.speed < 0.01, "and the same motor moves it nowhere once the limit has it");
}

// 3 and -3 are exactly representable and this joint bounds neither, so the swap shows up exactly.
fn void a_reversed_pair_of_translation_limits_is_swapped_into_order_rather_than_refused() @test {
    SliderLimits limits = the_limits_of_a_slider_set_to(3.0, -3.0);

    assert(limits.lower == -3.0, "box3d takes the smaller of the two as the lower");
    assert(limits.upper == 3.0);
}

// The pair above sits inside the 3.110177 radians the revolute joint clamps its own limits to, so
// the swap alone does not tell the two setters apart. Ten does.
fn void translation_limits_far_out_are_left_where_they_were_put() @test {
    SliderLimits limits = the_limits_of_a_slider_set_to(10.0, -10.0);

    assert(limits.lower == -10.0, "swapped the way the revolute joint's are, and not clamped the way those are");
    assert(limits.upper == 10.0);
}

// The other half of that asymmetry: the constructor applies neither correction. box3d asserts the
// definition's pair is already ordered and this release build compiles that assertion out.
fn void a_definition_that_reverses_its_two_translations_builds_a_slider_that_keeps_them_reversed() @test {
    SliderLimits limits = the_limits_of_a_slider_whose_definition_reversed_them();

    assert(limits.lower == 1.0, "the definition's lower translation, above its upper one, arrives intact");
    assert(limits.upper == -1.0, "where the same pair through set_limits comes back ordered");
}

fn void the_limit_flag_of_a_slider_starts_off_and_round_trips() @test {
    SliderLimitFlags flags = the_limit_flags_of_a_bare_slider_and_a_limited_one();

    assert(!flags.on_a_default_slider, "a default prismatic joint is a bare slider");
    assert(flags.from_the_definition, "the joint reports the limit its definition asked for");
    assert(flags.after_enabling);
    assert(!flags.after_disabling);
}

// Gravity off and no limit, so the spring is the only thing acting along the axis and the
// translation the slider comes to rest at is the target it was given. Five seconds at 1/60 s is
// well past the point it settles: doubling the run leaves both targets at the same reading.
fn float the_translation_a_sprung_slider_settles_at(bool enable_spring, float target) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId sliding = a_small_dynamic_sphere_at_the_origin(world);

    b3::PrismaticJointDef joint_def = b3::default_prismatic_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = sliding;
    joint_def.enable_spring = enable_spring;
    joint_def.hertz = 2.0;
    joint_def.damping_ratio = 1.0;
    joint_def.target_translation = target;
    b3::PrismaticJointId slider = world.create_prismatic_joint(&joint_def)!!;

    for (int i = 0; i < 300; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return slider.translation();
}

struct SliderSpringSettings {
    bool enabled;
    float hertz;
    float damping_ratio;
    float target;
    bool enabled_after;
    float hertz_after;
    float damping_ratio_after;
    float target_after;
}

fn SliderSpringSettings the_spring_settings_of_a_slider_before_and_after_the_setters() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId sliding = a_small_dynamic_sphere_at_the_origin(world);

    b3::PrismaticJointDef joint_def = b3::default_prismatic_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = sliding;
    joint_def.enable_spring = true;
    joint_def.hertz = 4.0;
    joint_def.damping_ratio = 0.5;
    joint_def.target_translation = 4.5;
    b3::PrismaticJointId slider = world.create_prismatic_joint(&joint_def)!!;

    SliderSpringSettings settings = {
        .enabled = slider.is_spring_enabled(),
        .hertz = slider.spring_hertz(),
        .damping_ratio = slider.spring_damping_ratio(),
        .target = slider.target_translation(),
    };

    slider.enable_spring(false);
    slider.set_spring_hertz(7.5);
    slider.set_spring_damping_ratio(0.125);
    slider.set_target_translation(-4.5);

    settings.enabled_after = slider.is_spring_enabled();
    settings.hertz_after = slider.spring_hertz();
    settings.damping_ratio_after = slider.spring_damping_ratio();
    settings.target_after = slider.target_translation();
    return settings;
}

struct SliderMotorDrive {
    float motor_force;
    b3::Vec3 constraint_force;
    float translation;
    float speed;
}

// Gravity off, no spring and no limit, so the motor is the only thing acting on the slider and
// the force it reports is its own. The 5 m/s it asks for is far more than two seconds of either
// ceiling buys against the body's mass, so both driven rows are still saturated at the end rather
// than coasting at the speed they were asked for.
fn SliderMotorDrive a_slider_motor_driven_to(bool enable_motor, float max_motor_force) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId sliding = a_small_dynamic_sphere_at_the_origin(world);

    b3::PrismaticJointDef joint_def = b3::default_prismatic_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = sliding;
    joint_def.enable_motor = enable_motor;
    joint_def.motor_speed = 5.0;
    joint_def.max_motor_force = max_motor_force;
    b3::PrismaticJointId slider = world.create_prismatic_joint(&joint_def)!!;

    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return {
        .motor_force = slider.motor_force(),
        .constraint_force = slider.constraint_force(),
        .translation = slider.translation(),
        .speed = slider.speed(),
    };
}

struct SliderMotorSettings {
    bool enabled;
    float speed;
    float max_force;
    bool enabled_after;
    float speed_after;
    float max_force_after;
}

fn SliderMotorSettings the_motor_settings_of_a_slider_before_and_after_the_setters() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId sliding = a_small_dynamic_sphere_at_the_origin(world);

    b3::PrismaticJointDef joint_def = b3::default_prismatic_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = sliding;
    joint_def.enable_motor = true;
    joint_def.motor_speed = 0.5;
    joint_def.max_motor_force = 100.0;
    b3::PrismaticJointId slider = world.create_prismatic_joint(&joint_def)!!;

    SliderMotorSettings settings = {
        .enabled = slider.is_motor_enabled(),
        .speed = slider.motor_speed(),
        .max_force = slider.max_motor_force(),
    };

    slider.enable_motor(false);
    slider.set_motor_speed(-0.25);
    slider.set_max_motor_force(12.5);

    settings.enabled_after = slider.is_motor_enabled();
    settings.speed_after = slider.motor_speed();
    settings.max_force_after = slider.max_motor_force();
    return settings;
}

struct SliderAxialForces {
    float settled;
    float after_the_motor_flag;
    float after_the_limit_flag;
    float after_the_spring_flag;
}

// One settled spring read four times, each reading after a flag has been toggled to the other
// value and back, and no step anywhere between them — so the flag is the only thing that can move
// the number. The motor and limit flags come first: they discard their own accumulated impulses,
// which are zero on this joint, so they leave the spring's alone and give the spring flag's zero
// its meaning.
fn SliderAxialForces the_axial_force_of_a_sprung_slider_around_toggling_its_flags() {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, -10.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId anchor = a_static_sphere_at_the_origin(world);
    b3::BodyId sliding = a_small_dynamic_sphere_at_the_origin(world);

    b3::PrismaticJointDef joint_def = b3::default_prismatic_joint_def();
    joint_def.body_id_a = anchor;
    joint_def.body_id_b = sliding;
    joint_def.enable_spring = true;
    joint_def.hertz = 2.0;
    joint_def.damping_ratio = 1.0;
    joint_def.target_translation = 0.7;
    b3::PrismaticJointId slider = world.create_prismatic_joint(&joint_def)!!;

    for (int i = 0; i < 300; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    SliderAxialForces forces = {.settled = slider.constraint_force().z};

    slider.enable_motor(true);
    slider.enable_motor(false);
    forces.after_the_motor_flag = slider.constraint_force().z;

    slider.enable_limit(true);
    slider.enable_limit(false);
    forces.after_the_limit_flag = slider.constraint_force().z;

    slider.enable_spring(false);
    slider.enable_spring(true);
    forces.after_the_spring_flag = slider.constraint_force().z;
    return forces;
}

// The spring pulls the slider onto the translation it was given and holds it there. 0.699962
// rather than exactly 0.7 because a spring of finite stiffness sits a little short of its target.
fn void a_spring_settles_the_slider_on_its_target_translation() @test {
    float settled = the_translation_a_sprung_slider_settles_at(true, 0.7);

    assert(settled > 0.69f && settled < 0.71f, "the slider comes to rest on the target the spring pulls it toward");
}

// The second target, on the other side of the origin, with a band disjoint from the first: what
// the two together pin is the target rather than a translation the joint would reach anyway.
fn void a_second_target_settles_the_slider_somewhere_else() @test {
    float settled = the_translation_a_sprung_slider_settles_at(true, -0.4);

    assert(settled > -0.41f && settled < -0.39f, "a target the other side of the origin is followed the same way");
}

// The control for both: the same definitions with only the spring flag cleared. The target the
// joint still carries pulls on nothing, so the slider stays where it was built, and that band is
// disjoint from both settled ones.
fn void a_slider_whose_spring_is_disabled_stays_where_it_was_built() @test {
    float forward = the_translation_a_sprung_slider_settles_at(false, 0.7);
    float back = the_translation_a_sprung_slider_settles_at(false, -0.4);

    assert(forward > -0.01f && forward < 0.01f, "a disabled spring pulls the slider toward nothing");
    assert(back > -0.01f && back < 0.01f);
}

// 4, 0.5, 4.5, 7.5, 0.125 and -4.5 are all exactly representable as floats, so they compare
// exactly. The two targets are both outside the [-pi, pi] the revolute joint's constructor clamps
// its own target into, and one arrives through each path here: neither bounds it.
fn void the_spring_settings_of_a_slider_round_trip() @test {
    SliderSpringSettings settings = the_spring_settings_of_a_slider_before_and_after_the_setters();

    assert(settings.enabled, "the joint reports the spring its definition asked for");
    assert(settings.hertz == 4.0);
    assert(settings.damping_ratio == 0.5);
    assert(settings.target == 4.5);
    assert(!settings.enabled_after, "and then the values the setters gave it");
    assert(settings.hertz_after == 7.5);
    assert(settings.damping_ratio_after == 0.125);
    assert(settings.target_after == -4.5);
}

// Two ceilings on the same fixture, each read back as the force the motor is applying, with bands
// disjoint from each other and from the disabled control. What the pair pins is the ceiling rather
// than a force the motor would settle at whatever it was allowed.
fn void the_motor_force_saturates_at_the_ceiling_it_was_given() @test {
    SliderMotorDrive light = a_slider_motor_driven_to(true, 10.0);
    SliderMotorDrive heavy = a_slider_motor_driven_to(true, 25.0);

    assert(light.motor_force > 9.9f && light.motor_force < 10.1f, "the motor is held at the ceiling it was given");
    assert(heavy.motor_force > 24.9f && heavy.motor_force < 25.1f, "and a higher ceiling is held at that instead");
    assert(light.translation > 0.29f && light.translation < 0.32f,
        "the lighter ceiling carries the slider a third of a metre in two seconds");
    assert(heavy.translation > 0.75f && heavy.translation < 0.78f,
        "and the heavier one carries it more than twice as far in the same time");
}

// The control for the pair above: the same definition with only the motor flag cleared, so the
// speed and the ceiling the joint still carries drive nothing.
fn void a_slider_whose_motor_is_disabled_applies_no_motor_force() @test {
    SliderMotorDrive idle = a_slider_motor_driven_to(false, 25.0);

    assert(idle.motor_force > -0.001f && idle.motor_force < 0.001f, "a disabled motor applies nothing");
    assert(idle.translation > -0.001f && idle.translation < 0.001f, "and the slider stays where it was built");
    assert(idle.speed > -0.001f && idle.speed < 0.001f);
}

// 0.5, 100, -0.25 and 12.5 are all exactly representable as floats, so they compare exactly.
fn void the_motor_settings_of_a_slider_round_trip() @test {
    SliderMotorSettings settings = the_motor_settings_of_a_slider_before_and_after_the_setters();

    assert(settings.enabled, "the joint reports the motor its definition asked for");
    assert(settings.speed == 0.5);
    assert(settings.max_force == 100.0);
    assert(!settings.enabled_after, "and then the values the setters gave it");
    assert(settings.speed_after == -0.25);
    assert(settings.max_force_after == 12.5);
}

// The relationship a consumer reaching for the shared accessor needs, and it is neither of the two
// the earlier joint types show. box3d sums this joint's axial impulses once, the way the distance
// joint's are summed and not twice the way the revolute joint's are, so the magnitudes agree — but
// it lays the total along local frame A's z axis while the slider runs along that frame's x axis.
// The frames are identity here, so the body is moving along world x under a force reported along
// world z, and the two components below are what say so.
fn void the_shared_constraint_force_carries_the_motor_force_across_the_sliding_axis() @test {
    SliderMotorDrive driven = a_slider_motor_driven_to(true, 25.0);

    assert(driven.translation > 0.75f && driven.translation < 0.78f, "the slider is moving along the joint's x axis");
    assert(driven.constraint_force.x > -0.001f && driven.constraint_force.x < 0.001f,
        "and none of the force the shared accessor reports lies along it");
    assert(driven.constraint_force.y > -0.001f && driven.constraint_force.y < 0.001f);

    float ratio = driven.constraint_force.z / driven.motor_force;
    assert(ratio > 0.999f && ratio < 1.001f, "the whole of it lies along z, at the motor force counted once");
}

// Same accessor name, different contract, across joint types: this joint's spring flag discards
// the impulse the spring had accumulated where the distance joint's leaves it standing. The two
// controls are what make this discriminate — they are the same shape of call on the same joint, so
// a reading that survives them and then falls to zero can only have been the spring flag.
fn void toggling_the_spring_discards_the_force_it_had_where_the_other_two_flags_do_not() @test {
    SliderAxialForces forces = the_axial_force_of_a_sprung_slider_around_toggling_its_flags();

    assert(forces.settled > -0.26f && forces.settled < -0.24f,
        "the settled spring is holding the slider on its target");
    assert(forces.after_the_motor_flag == forces.settled,
        "toggling the motor leaves the spring's accumulated force alone");
    assert(forces.after_the_limit_flag == forces.settled, "and so does toggling the limit");
    assert(forces.after_the_spring_flag > -0.001f && forces.after_the_spring_flag < 0.001f,
        "while toggling the spring throws that force away, before any step could rebuild it");
}
