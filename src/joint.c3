module b3;

<*
 Links two bodies with a joint that constrains nothing, so that they stop colliding.

 collide_connected does the suppressing, not the type, so set_collide_connected(true) puts the
 collision back; box3d islands the pair only while one body is dynamic and neither is disabled.

 @param [&in] def : `a definition obtained from default_filter_joint_def(), naming two bodies of
   this world`
 @return `the joint`
 @return? check_joint_def!
 @return? INVALID_HANDLE : `the world is locked, mid step or inside a callback box3d runs during
   one`
*>
fn JointId? WorldId.create_filter_joint(self, FilterJointDef* def) {
    if (def == null) return INVALID_DEFINITION~;
    check_joint_def(self, &def.base)!;
    return create_filter_joint_raw(self, def).checked();
}

<*
 The soft-constraint tuning this joint is solved with.

 @return `the stiffness in hertz and the damping ratio, 60 and 2 on a joint built from a default
   definition`
*>
fn ConstraintTuning JointId.constraint_tuning(self) {
    ConstraintTuning tuning;
    constraint_tuning_raw(self, &tuning.hertz, &tuning.damping_ratio);
    return tuning;
}

<*
 Writes the joints linked to this body into the caller's buffer, most recently created first:
 box3d inserts each new joint at the head of the list. joint_count() is exact, so a buffer that
 long is never truncated, and the fill keeps working while the world is locked, unlike the sensor
 filler.

 @return `the prefix of buffer that was filled`
*>
fn JointId[] BodyId.joints(self, JointId[] buffer) {
    int written = body_joints_raw(self, buffer.ptr, (int)buffer.len);
    return buffer[:written];
}

<*
 The one implementation of the checks every joint constructor runs. b3Body_IsValid answers for the
 world an identifier came from rather than the world the call was handed, so the two worlds are
 compared separately, and the world's own generation test is an assertion this release build
 compiles out — without the first line a handle to a destroyed world would reach whatever has
 since taken its slot.

 @param [&in] def : `the base embedded in the caller's definition`
 @return? INVALID_DEFINITION : `this world no longer exists, the definition is unmarked, or it
   names a body this world cannot resolve`
*>
fn void? check_joint_def(WorldId world_id, JointDef* def) @private {
    if (!world_id.is_valid()) return INVALID_DEFINITION~;
    if (def.internal_value != default_filter_joint_def().internal_value) return INVALID_DEFINITION~;
    if (!def.body_id_a.is_valid() || !def.body_id_b.is_valid()) return INVALID_DEFINITION~;

    int world0 = world_id.index1 - 1;
    if (def.body_id_a.world0 != world0 || def.body_id_b.world0 != world0) return INVALID_DEFINITION~;
}

<*
 The one implementation of the checks every joint narrowing runs. A narrowed handle's accessors
 resolve through a joint-type assertion this release build compiles out, so one reaching a joint
 of another type reinterprets that joint's solver union — nonsense from a getter, an overwrite
 from a setter. Liveness is tested first: a destroyed joint's type resolves through that same
 compiled-out check.

 @return? INVALID_HANDLE : `the joint no longer exists`
 @return? WRONG_JOINT_TYPE : `the joint is of another type`
*>
fn void? check_joint_type(JointId joint, JointType type) @private {
    if (!joint.is_valid()) return INVALID_HANDLE~;
    if (joint.type() != type) return WRONG_JOINT_TYPE~;
}
