module b3_test;

import b3;
import std::math;

fn void the_mover_and_pogo_joint_types_match_the_c_layout() @test {
    assert(b3::MoverJointDef::size == 128);
    assert(b3::MoverJointDef::alignment == 8);
    assert(b3::PogoJointDef::size == 152);
    assert(b3::PogoJointDef::alignment == 8);
    assert((int)b3::JointType.MOVER == 9);
    assert((int)b3::JointType.POGO == 10);
}

// 0 and 1 are exactly representable as floats and are copied straight out of the definition, so
// these compare exactly rather than in a band.
fn void the_default_mover_joint_definition_zeroes_both_fields_it_adds_to_the_base() @test {
    b3::MoverJointDef def = b3::default_mover_joint_def();

    assert(def.linear_velocity.x == 0.0 && def.linear_velocity.y == 0.0 && def.linear_velocity.z == 0.0);
    assert(def.max_force == 0.0, "a default mover drives nothing");
}

fn void the_default_pogo_joint_definition_points_up_and_zeroes_the_rest() @test {
    b3::PogoJointDef def = b3::default_pogo_joint_def();

    assert(def.normal.x == 0.0 && def.normal.y == 1.0 && def.normal.z == 0.0);
    assert(def.hertz == 0.0, "a default pogo pushes nothing");
    assert(def.damping_ratio == 0.0);
    assert(def.rest_length == 0.0);
    assert(def.max_tension_force == 0.0);
    assert(def.max_compression_force == 0.0);
    assert(def.impulse == 0.0);
    assert(def.velocity == 0.0);
}

fn void the_default_mover_and_pogo_definitions_carry_the_cookie_every_definition_shares() @test {
    b3::MoverJointDef mover_def = b3::default_mover_joint_def();
    b3::PogoJointDef pogo_def = b3::default_pogo_joint_def();
    b3::FilterJointDef filter_def = b3::default_filter_joint_def();

    assert(mover_def.internal_value == mover_def.base.internal_value, "the direct field names the embedded base");
    assert(mover_def.internal_value == filter_def.internal_value, "every definition starts from the same base");
    assert(pogo_def.internal_value == filter_def.internal_value);
}

fn b3::WorldId a_world_with_gravity(float gravity) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, gravity, 0.0 };
    return b3::create_world(&world_def)!!;
}

fn b3::BodyId a_static_anchor(b3::WorldId world) {
    b3::BodyDef body_def = b3::default_body_def();
    return b3::create_body(world, &body_def)!!;
}

// Density 1 rather than the default 1000 gives a sphere of about half a kilogram, so the force
// ceilings below stay small round numbers.
fn b3::BodyId a_dynamic_sphere_at_height(b3::WorldId world, float y) {
    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, y, 0.0};
    b3::BodyId body = b3::create_body(world, &body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    b3::create_sphere_shape(body, &shape_def, &sphere)!!;
    return body;
}

fn void step_seconds(b3::WorldId world, int steps) {
    for (int i = 0; i < steps; i++) {
        b3::world_step(world, 1.0f / 60.0f, 4);
    }
}

fn bool lies_between(float value, float low, float high) => value >= low && value <= high;

struct MoverOutcome {
    float mass;
    b3::Vec3 target;
    float ceiling;
    b3::Vec3 velocity;
    b3::Vec3 angular_velocity;
    b3::Vec3 force;
    b3::Vec3 torque;
    float linear_separation;
    float angular_separation;
    bool awake;
}

// The sphere floats free with no gravity, so the only thing moving it is the mover. The world is
// destroyed before the caller asserts, so a failing assertion cannot strand it.
fn MoverOutcome a_mover_after(b3::Vec3 target, float ceiling, b3::Quat frame_b, int steps) {
    b3::WorldId world = a_world_with_gravity(0.0);
    defer b3::destroy_world(world);

    b3::BodyId ground = a_static_anchor(world);
    b3::BodyId body = a_dynamic_sphere_at_height(world, 5.0);

    b3::MoverJointDef def = b3::default_mover_joint_def();
    def.body_id_a = ground;
    def.body_id_b = body;
    def.local_frame_b.q = frame_b;
    b3::MoverJointId mover = b3::create_mover_joint(world, &def)!!;
    mover.set_linear_velocity(target);
    mover.set_max_velocity_force(ceiling);

    step_seconds(world, steps);

    return {
        .mass = body.mass(),
        .target = mover.linear_velocity(),
        .ceiling = mover.max_velocity_force(),
        .velocity = body.linear_velocity(),
        .angular_velocity = body.angular_velocity(),
        .force = mover.constraint_force(),
        .torque = mover.constraint_torque(),
        .linear_separation = mover.linear_separation(),
        .angular_separation = mover.angular_separation(),
        .awake = mover.is_awake(),
    };
}

const b3::Quat IDENTITY = {0.0, 0.0, 0.0, 1.0};
// A quarter turn about x carries local frame B's z axis onto world -y.
const b3::Quat QUARTER_TURN_ABOUT_X = {0.70710678, 0.0, 0.0, 0.70710678};
// A half turn about z carries local frame B's y axis onto world -y.
const b3::Quat HALF_TURN_ABOUT_Z = {0.0, 0.0, 1.0, 0.0};

fn void a_mover_drives_the_body_to_its_target_in_the_xz_plane_and_ignores_y() @test {
    MoverOutcome outcome = a_mover_after({1.0, 2.0, -3.0}, 1000.0, IDENTITY, 30);

    assert(outcome.target.x == 1.0 && outcome.target.y == 2.0 && outcome.target.z == -3.0,
        "the getter reports the whole target, the ignored y included");
    assert(outcome.ceiling == 1000.0);
    assert(lies_between(outcome.velocity.x, 0.99, 1.01));
    assert(lies_between(outcome.velocity.y, -0.01, 0.01), "frame B's y axis carries no drive");
    assert(lies_between(outcome.velocity.z, -3.01, -2.99));
}

// Exact zero: the mover applies its impulse at the centre of mass with no angular term, and
// nothing else touches the sphere.
fn void a_mover_leaves_the_body_unturned_and_reports_no_torque_or_separation() @test {
    MoverOutcome outcome = a_mover_after({1.0, 0.0, 0.0}, 1000.0, IDENTITY, 30);

    assert(outcome.angular_velocity.x == 0.0 && outcome.angular_velocity.y == 0.0
        && outcome.angular_velocity.z == 0.0);
    assert(outcome.torque.x == 0.0 && outcome.torque.y == 0.0 && outcome.torque.z == 0.0);
    assert(outcome.linear_separation == 0.0);
    assert(outcome.angular_separation == 0.0);
    assert(outcome.awake, "a joint on a moving dynamic body is in the awake set");
}

fn void a_mover_drives_along_local_frame_b_rather_than_the_world_axes() @test {
    MoverOutcome outcome = a_mover_after({1.0, 2.0, -3.0}, 1000.0, QUARTER_TURN_ABOUT_X, 30);

    assert(lies_between(outcome.velocity.x, 0.99, 1.01));
    assert(lies_between(outcome.velocity.y, 1.99, 2.01), "frame B's z axis now lies along world y");
    assert(lies_between(outcome.velocity.z, -0.01, 0.01), "and world z is now the axis it ignores");
}

// A zero ceiling clamps every impulse to exactly zero, so the sphere does not move at all.
fn void a_mover_with_a_zero_ceiling_drives_nothing() @test {
    MoverOutcome outcome = a_mover_after({1.0, 0.0, 0.0}, 0.0, IDENTITY, 30);

    assert(outcome.velocity.x == 0.0 && outcome.velocity.y == 0.0 && outcome.velocity.z == 0.0);
    assert(outcome.force.x == 0.0 && outcome.force.y == 0.0 && outcome.force.z == 0.0);
}

// A 1 N ceiling cannot bring a half-kilogram sphere to 10 m/s within one second, so the drive is
// saturated throughout: it reports its ceiling and accelerates the sphere at ceiling / mass.
fn void a_saturated_mover_reports_its_ceiling_as_its_constraint_force() @test {
    MoverOutcome outcome = a_mover_after({10.0, 0.0, 0.0}, 1.0, IDENTITY, 60);

    assert(lies_between(outcome.force.x, 0.99, 1.01));
    assert(lies_between(outcome.force.y, -0.01, 0.01));
    assert(lies_between(outcome.force.z, -0.01, 0.01));
    float expected = 1.0f / outcome.mass;
    assert(lies_between(outcome.velocity.x, 0.98f * expected, 1.02f * expected));
}

fn void a_negative_ceiling_pushes_a_saturated_mover_away_from_its_target() @test {
    MoverOutcome outcome = a_mover_after({10.0, 0.0, 0.0}, -1.0, IDENTITY, 60);

    assert(outcome.ceiling == -1.0, "stored as handed");
    float expected = -1.0f / outcome.mass;
    assert(lies_between(outcome.velocity.x, 1.02f * expected, 0.98f * expected));
}

fn void a_negative_ceiling_drives_an_unsaturated_mover_as_its_magnitude_would() @test {
    MoverOutcome outcome = a_mover_after({1.0, 0.0, 0.0}, -1000.0, IDENTITY, 30);

    assert(lies_between(outcome.velocity.x, 0.99, 1.01));
}

struct DroppedCeiling {
    float speed_when_dropped;
    float speed_after;
    b3::Vec3 force_after;
}

fn DroppedCeiling a_mover_whose_ceiling_drops_to_zero_mid_run() {
    b3::WorldId world = a_world_with_gravity(0.0);
    defer b3::destroy_world(world);

    b3::BodyId ground = a_static_anchor(world);
    b3::BodyId body = a_dynamic_sphere_at_height(world, 5.0);

    b3::MoverJointDef def = b3::default_mover_joint_def();
    def.body_id_a = ground;
    def.body_id_b = body;
    def.linear_velocity = {10.0, 0.0, 0.0};
    def.max_force = 1.0;
    b3::MoverJointId mover = b3::create_mover_joint(world, &def)!!;

    step_seconds(world, 30);
    float speed_when_dropped = body.linear_velocity().x;
    mover.set_max_velocity_force(0.0);
    step_seconds(world, 30);

    return {
        .speed_when_dropped = speed_when_dropped,
        .speed_after = body.linear_velocity().x,
        .force_after = mover.constraint_force(),
    };
}

// The impulse the drive held is warm-started back in and cancelled within the same sub-step, so
// the sphere coasts at the speed it had rather than being pushed on.
fn void dropping_a_mover_ceiling_to_zero_leaves_the_body_coasting() @test {
    DroppedCeiling outcome = a_mover_whose_ceiling_drops_to_zero_mid_run();

    assert(outcome.speed_when_dropped > 0.5, "the drive had worked before the drop");
    assert(lies_between(outcome.speed_after, outcome.speed_when_dropped - 0.001, outcome.speed_when_dropped + 0.001));
    assert(outcome.force_after.x == 0.0);
}

struct PogoOutcome {
    float mass;
    float height;
    float vertical_speed;
    float current_length;
    float current_speed;
    float impulse;
    b3::Vec3 force;
    b3::Vec3 torque;
    float linear_separation;
}

fn PogoOutcome pogo_outcome(b3::PogoJointId pogo, b3::BodyId body) {
    return {
        .mass = body.mass(),
        .height = body.position().y,
        .vertical_speed = body.linear_velocity().y,
        .current_length = pogo.current_length(),
        .current_speed = pogo.current_speed(),
        .impulse = pogo.impulse(),
        .force = pogo.constraint_force(),
        .torque = pogo.constraint_torque(),
        .linear_separation = pogo.linear_separation(),
    };
}

// Anchor A sits 1 m above the static body and the sphere starts 3 m up, so the pogo starts at its
// 2 m rest length. Both ceilings carry a hundred times the sphere's weight.
fn b3::PogoJointDef a_sprung_pogo_def(b3::BodyId ground, b3::BodyId body) {
    b3::PogoJointDef def = b3::default_pogo_joint_def();
    def.body_id_a = ground;
    def.body_id_b = body;
    def.local_frame_a.p = {0.0, 1.0, 0.0};
    def.rest_length = 2.0;
    def.hertz = 2.0;
    def.damping_ratio = 1.0;
    def.max_compression_force = 100.0f * 10.0f * body.mass();
    def.max_tension_force = 100.0f * 10.0f * body.mass();
    return def;
}

fn PogoOutcome a_sprung_pogo_after(int steps) {
    b3::WorldId world = a_world_with_gravity(-10.0);
    defer b3::destroy_world(world);

    b3::BodyId ground = a_static_anchor(world);
    b3::BodyId body = a_dynamic_sphere_at_height(world, 3.0);
    b3::PogoJointDef def = a_sprung_pogo_def(ground, body);
    b3::PogoJointId pogo = b3::create_pogo_joint(world, &def)!!;

    step_seconds(world, steps);
    return pogo_outcome(pogo, body);
}

// At rest the spring carries the sphere's weight, so the force it reports is m g upward. The
// impulse is that force over one 1/240 s sub-step.
fn void a_pogo_holds_the_body_at_its_rest_length_against_gravity() @test {
    PogoOutcome outcome = a_sprung_pogo_after(300);
    float weight = 10.0f * outcome.mass;

    assert(lies_between(outcome.height, 2.98, 3.02));
    assert(lies_between(outcome.current_length, 1.98, 2.02));
    assert(lies_between(outcome.vertical_speed, -0.01, 0.01));
    assert(lies_between(outcome.current_speed, -0.01, 0.01), "the spring has nothing left to recover");
    assert(lies_between(outcome.force.y, 0.98f * weight, 1.02f * weight));
    assert(lies_between(outcome.impulse * 240.0f, 0.98f * weight, 1.02f * weight), "positive pushes body B away");
}

fn void a_pogo_reports_no_torque_and_no_separation_while_it_works() @test {
    PogoOutcome outcome = a_sprung_pogo_after(300);

    assert(outcome.force.y > 1.0, "the spring is carrying the sphere");
    assert(outcome.torque.x == 0.0 && outcome.torque.y == 0.0 && outcome.torque.z == 0.0);
    assert(outcome.linear_separation == 0.0);
}

fn PogoOutcome a_default_pogo_after(int steps) {
    b3::WorldId world = a_world_with_gravity(-10.0);
    defer b3::destroy_world(world);

    b3::BodyId ground = a_static_anchor(world);
    b3::BodyId body = a_dynamic_sphere_at_height(world, 3.0);
    b3::PogoJointDef def = b3::default_pogo_joint_def();
    def.body_id_a = ground;
    def.body_id_b = body;
    def.local_frame_a.p = {0.0, 1.0, 0.0};
    def.impulse = 0.25;
    def.velocity = -0.5;
    b3::PogoJointId pogo = b3::create_pogo_joint(world, &def)!!;

    step_seconds(world, steps);
    return pogo_outcome(pogo, body);
}

// Exact: the seeds are copied verbatim and nothing has stepped yet.
fn void a_new_pogo_reports_the_impulse_and_velocity_its_definition_seeded() @test {
    PogoOutcome outcome = a_default_pogo_after(0);

    assert(outcome.impulse == 0.25);
    assert(outcome.current_speed == -0.5);
}

// A zero hertz clears both pieces of spring state on the first sub-step, seeds included, and the
// sphere falls freely: v = g t after half a second.
fn void a_default_pogo_lets_the_body_fall_and_clears_its_seeded_state() @test {
    PogoOutcome outcome = a_default_pogo_after(30);

    assert(outcome.impulse == 0.0);
    assert(outcome.current_speed == 0.0);
    assert(lies_between(outcome.vertical_speed, -5.05, -4.95));
}

struct PogoLengths {
    float upright;
    float flipped;
}

// Exact up to rounding: identity rotations, and every coordinate is a small integer.
fn PogoLengths the_lengths_of_two_new_pogos() {
    b3::WorldId world = a_world_with_gravity(-10.0);
    defer b3::destroy_world(world);

    b3::BodyId ground = a_static_anchor(world);
    b3::BodyId body = a_dynamic_sphere_at_height(world, 3.0);
    b3::PogoJointDef def = a_sprung_pogo_def(ground, body);
    b3::PogoJointId upright = b3::create_pogo_joint(world, &def)!!;
    def.local_frame_b.q = HALF_TURN_ABOUT_Z;
    b3::PogoJointId flipped = b3::create_pogo_joint(world, &def)!!;

    return {.upright = upright.current_length(), .flipped = flipped.current_length()};
}

fn void a_pogo_measures_its_length_along_local_frame_b_y_axis() @test {
    PogoLengths lengths = the_lengths_of_two_new_pogos();

    assert(lies_between(lengths.upright, 1.9999, 2.0001));
    assert(lies_between(lengths.flipped, -2.0001, -1.9999), "the flipped frame measures down its own y axis");
}

struct PogoSettings {
    float rest_length;
    float hertz;
    float damping_ratio;
    float max_tension_force;
    float max_compression_force;
    b3::Vec3 normal;
    float settled_length;
}

// Every value differs from what the definition set, so a setter that stored nothing would fail.
fn PogoSettings a_pogo_after_every_setter() {
    b3::WorldId world = a_world_with_gravity(-10.0);
    defer b3::destroy_world(world);

    b3::BodyId ground = a_static_anchor(world);
    b3::BodyId body = a_dynamic_sphere_at_height(world, 3.0);
    b3::PogoJointDef def = a_sprung_pogo_def(ground, body);
    b3::PogoJointId pogo = b3::create_pogo_joint(world, &def)!!;

    pogo.set_spring_hertz(3.0);
    pogo.set_spring_damping_ratio(0.9);
    pogo.set_max_tension_force(50.0);
    pogo.set_max_compression_force(60.0);
    pogo.set_normal({0.0, 0.0, 1.0});
    PogoSettings settings = {
        .hertz = pogo.spring_hertz(),
        .damping_ratio = pogo.spring_damping_ratio(),
        .max_tension_force = pogo.max_tension_force(),
        .max_compression_force = pogo.max_compression_force(),
        .normal = pogo.normal(),
    };

    pogo.set_normal({0.0, 1.0, 0.0});
    pogo.set_rest_length(1.5);
    settings.rest_length = pogo.rest_length();
    step_seconds(world, 300);
    settings.settled_length = pogo.current_length();
    return settings;
}

fn void the_pogo_settings_round_trip_and_a_new_rest_length_moves_the_body() @test {
    PogoSettings settings = a_pogo_after_every_setter();

    assert(settings.hertz == 3.0);
    assert(lies_between(settings.damping_ratio, 0.8999, 0.9001));
    assert(settings.max_tension_force == 50.0);
    assert(settings.max_compression_force == 60.0);
    assert(settings.normal.x == 0.0 && settings.normal.y == 0.0 && settings.normal.z == 1.0);
    assert(settings.rest_length == 1.5);
    assert(lies_between(settings.settled_length, 1.48, 1.52), "the spring settled on the new rest length");
}

struct JointAwakeness {
    bool awake_when_created;
    bool asleep_bodies;
    bool awake_while_asleep;
    bool awake_after_waking;
    bool awake_with_a_disabled_body;
    bool static_pair_awake;
}

fn JointAwakeness the_awakeness_of_a_joint_through_sleep() {
    b3::WorldId world = a_world_with_gravity(0.0);
    defer b3::destroy_world(world);

    b3::BodyId a = a_dynamic_sphere_at(world, -3.0);
    b3::BodyId b = a_dynamic_sphere_at(world, 3.0);
    b3::JointId joint = link_with_a_filter_joint(world, a, b);
    JointAwakeness awakeness = {.awake_when_created = joint.is_awake()};

    step_seconds(world, 120);
    awakeness.asleep_bodies = !a.is_awake() && !b.is_awake();
    awakeness.awake_while_asleep = joint.is_awake();

    b3::joint_wake_bodies(joint);
    awakeness.awake_after_waking = joint.is_awake();

    b3::body_disable(b);
    awakeness.awake_with_a_disabled_body = joint.is_awake();

    b3::JointId static_pair = link_with_a_filter_joint(world, a_static_anchor(world), a_static_anchor(world));
    awakeness.static_pair_awake = static_pair.is_awake();
    return awakeness;
}

fn void a_joint_is_awake_exactly_while_its_bodies_are() @test {
    JointAwakeness awakeness = the_awakeness_of_a_joint_through_sleep();

    assert(awakeness.awake_when_created);
    assert(awakeness.asleep_bodies, "two untouched spheres fall asleep within two seconds");
    assert(!awakeness.awake_while_asleep);
    assert(awakeness.awake_after_waking);
    assert(!awakeness.awake_with_a_disabled_body, "a disabled body takes its joints out of the awake set");
    assert(!awakeness.static_pair_awake, "a joint linking no dynamic body is never awake");
}

struct MoverPogoConversions {
    bool mover_converted;
    bool pogo_converted;
    b3::JointId mover;
    b3::JointId pogo;
    b3::JointId narrowed_mover;
    b3::JointId narrowed_pogo;
    fault mover_as_pogo;
    fault pogo_as_mover;
    fault filter_as_mover;
    fault filter_as_pogo;
    fault destroyed_mover;
    fault destroyed_pogo;
}

// All three joints link the same pair of bodies in the same world, so the only thing separating
// the joint each conversion refuses is its type, until the last two are destroyed.
fn MoverPogoConversions the_conversions_of_a_mover_a_pogo_and_a_filter_joint() {
    b3::WorldId world = a_world_with_gravity(0.0);
    defer b3::destroy_world(world);

    b3::BodyId ground = a_static_anchor(world);
    b3::BodyId body = a_dynamic_sphere_at_height(world, 3.0);

    b3::MoverJointDef mover_def = b3::default_mover_joint_def();
    mover_def.body_id_a = ground;
    mover_def.body_id_b = body;
    b3::JointId mover = b3::create_mover_joint(world, &mover_def)!!;

    b3::PogoJointDef pogo_def = b3::default_pogo_joint_def();
    pogo_def.body_id_a = ground;
    pogo_def.body_id_b = body;
    b3::JointId pogo = b3::create_pogo_joint(world, &pogo_def)!!;

    b3::JointId filter = link_with_a_filter_joint(world, ground, body);

    MoverPogoConversions conversions = {.mover = mover, .pogo = pogo};
    if (try narrowed = mover.as_mover()) {
        conversions.narrowed_mover = narrowed;
        conversions.mover_converted = true;
    }
    if (try narrowed = pogo.as_pogo()) {
        conversions.narrowed_pogo = narrowed;
        conversions.pogo_converted = true;
    }
    if (catch err = mover.as_pogo()) conversions.mover_as_pogo = err;
    if (catch err = pogo.as_mover()) conversions.pogo_as_mover = err;
    if (catch err = filter.as_mover()) conversions.filter_as_mover = err;
    if (catch err = filter.as_pogo()) conversions.filter_as_pogo = err;

    b3::destroy_joint(mover, false);
    b3::destroy_joint(pogo, false);
    if (catch err = mover.as_mover()) conversions.destroyed_mover = err;
    if (catch err = pogo.as_pogo()) conversions.destroyed_pogo = err;
    return conversions;
}

fn void a_mover_and_a_pogo_narrow_to_their_own_handles_and_no_other() @test {
    MoverPogoConversions conversions = the_conversions_of_a_mover_a_pogo_and_a_filter_joint();

    assert(conversions.mover_converted);
    assert(conversions.narrowed_mover.index1 == conversions.mover.index1);
    assert(conversions.narrowed_mover.generation == conversions.mover.generation);
    assert(conversions.pogo_converted);
    assert(conversions.narrowed_pogo.index1 == conversions.pogo.index1);
    assert(conversions.narrowed_pogo.generation == conversions.pogo.generation);
    assert(conversions.mover_as_pogo == b3::WRONG_JOINT_TYPE);
    assert(conversions.pogo_as_mover == b3::WRONG_JOINT_TYPE);
    assert(conversions.filter_as_mover == b3::WRONG_JOINT_TYPE);
    assert(conversions.filter_as_pogo == b3::WRONG_JOINT_TYPE);
}

fn void a_destroyed_joint_does_not_narrow_to_a_mover_or_pogo_handle() @test {
    MoverPogoConversions conversions = the_conversions_of_a_mover_a_pogo_and_a_filter_joint();

    assert(conversions.destroyed_mover == b3::INVALID_HANDLE, "liveness is tested before the type");
    assert(conversions.destroyed_pogo == b3::INVALID_HANDLE);
}

struct MoverPogoRefusals {
    fault zeroed_mover;
    fault zeroed_pogo;
    fault bodiless_mover;
    fault bodiless_pogo;
}

// Every refusal here is the wrapper's own, raised before box3d is reached, so none allocates.
fn MoverPogoRefusals the_refusals_of_four_bad_definitions() {
    b3::WorldId world = a_world_with_gravity(0.0);
    defer b3::destroy_world(world);

    b3::BodyId ground = a_static_anchor(world);
    b3::BodyId body = a_dynamic_sphere_at_height(world, 3.0);
    MoverPogoRefusals refusals;

    b3::MoverJointDef zeroed_mover;
    zeroed_mover.body_id_a = ground;
    zeroed_mover.body_id_b = body;
    if (catch err = b3::create_mover_joint(world, &zeroed_mover)) refusals.zeroed_mover = err;

    b3::PogoJointDef zeroed_pogo;
    zeroed_pogo.body_id_a = ground;
    zeroed_pogo.body_id_b = body;
    if (catch err = b3::create_pogo_joint(world, &zeroed_pogo)) refusals.zeroed_pogo = err;

    b3::MoverJointDef bodiless_mover = b3::default_mover_joint_def();
    if (catch err = b3::create_mover_joint(world, &bodiless_mover)) refusals.bodiless_mover = err;

    b3::PogoJointDef bodiless_pogo = b3::default_pogo_joint_def();
    if (catch err = b3::create_pogo_joint(world, &bodiless_pogo)) refusals.bodiless_pogo = err;
    return refusals;
}

fn void unmarked_or_bodiless_mover_and_pogo_definitions_are_refused() @test {
    MoverPogoRefusals refusals = the_refusals_of_four_bad_definitions();

    assert(refusals.zeroed_mover == b3::INVALID_DEFINITION);
    assert(refusals.zeroed_pogo == b3::INVALID_DEFINITION);
    assert(refusals.bodiless_mover == b3::INVALID_DEFINITION);
    assert(refusals.bodiless_pogo == b3::INVALID_DEFINITION);
}
