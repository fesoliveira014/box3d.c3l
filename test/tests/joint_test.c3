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

// The world is destroyed before the caller asserts anything, so a failing assertion cannot
// strand it: C3 skips defer on a panicking assertion and every later world or byte count in the
// run would then measure against a shifted baseline.
fn float[2] two_overlapping_spheres_after_sixty_steps(bool with_joint) {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = (b3::Vec3){ 0.0, 0.0, 0.0 };
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyId a = a_dynamic_sphere_at(world, -0.4);
    b3::BodyId b = a_dynamic_sphere_at(world, 0.4);
    if (with_joint) link_with_a_filter_joint(world, a, b);

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

// Two spheres of radius 0.5 whose centres start 0.8 apart overlap by 0.2, so they separate until
// they touch at 1.0 unless the pair is filtered out of collision. The control row is what makes
// the jointed row mean anything.
fn void a_filter_joint_stops_two_overlapping_bodies_pushing_apart() @test {
    float[2] unjointed = two_overlapping_spheres_after_sixty_steps(false);
    float gap = unjointed[1] - unjointed[0];
    assert(gap > 0.999 && gap < 1.001, "left to collide, the two spheres push apart until they touch");

    float[2] jointed = two_overlapping_spheres_after_sixty_steps(true);
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
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::FilterJointDef zeroed;
    if (catch err = world.create_filter_joint(&zeroed)) {
        assert(err == b3::INVALID_DEFINITION);
        return;
    }
    assert(false, "box3d accepts a zeroed joint definition; the binding must not");
}
