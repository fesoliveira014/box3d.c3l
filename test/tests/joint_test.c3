module b3_test;

import b3;

fn void the_joint_types_match_the_c_layout() @test {
    assert(b3::JointType::size == 4);
    assert(b3::JointDef::size == 112);
    assert(b3::JointDef::alignment == 8);
    assert(b3::FilterJointDef::size == 112);
    assert(b3::FilterJointDef::alignment == 8);
    assert(b3::ConstraintTuning::size == 8);
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

// 60, 2, 1 and 0 are exactly representable as floats, so they compare exactly rather than in a band.
fn void the_default_filter_joint_definition_carries_the_values_the_c_side_sets() @test {
    b3::FilterJointDef def = b3::default_filter_joint_def();

    assert(def.internal_value == 1152023);
    assert(def.force_threshold == float::max);
    assert(def.torque_threshold == float::max);
    assert(def.constraint_hertz == 60.0);
    assert(def.constraint_damping_ratio == 2.0);
    assert(def.draw_scale == 1.0);
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
