module b3;

<*
 Links two bodies with a joint that constrains nothing, so that they stop colliding.

 collide_connected does the suppressing, not the type, so set_collide_connected(true) puts the
 collision back; box3d islands the pair only while one body is dynamic and neither is disabled.
 The definition must come from default_filter_joint_def() and must name two bodies of this live
 world, checks box3d compiles out here: an unmarked one reports a joint event every awake step, an
 unresolvable identifier is indexed and written through, and frames and thresholds go unchecked.

 @param [&in] def : "a definition obtained from default_filter_joint_def(), naming two bodies of this world"
 @return "the joint, INVALID_DEFINITION if this world no longer exists, if the definition did not come from its type's default definition, or if it names a body this world cannot resolve, or INVALID_HANDLE if box3d returns a null identifier — which the type's constructor does only when the target world is locked, mid b3World_Step or inside a callback box3d runs during one"
*>
fn JointId? WorldId.create_filter_joint(self, FilterJointDef* def) {
    check_joint_def(self, &def.base)!;
    return create_filter_joint_raw(self, def).checked();
}

<*
 The soft-constraint tuning this joint is solved with.

 @return "the joint's stiffness in hertz and its damping ratio, 60 and 2 on a joint built from a default definition"
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

 @return "the prefix of buffer that was filled"
*>
fn JointId[] BodyId.joints(self, JointId[] buffer) {
    int written = body_joints_raw(self, buffer.ptr, (int)buffer.len);
    return buffer[:written];
}

<*
 The one implementation of the checks every joint constructor runs; WorldId.create_filter_joint
 says what they buy and where they stop. b3Body_IsValid answers for the world an identifier came
 from rather than the world the call was handed, so the two worlds are compared separately, and
 the world's own generation test is an assertion this release build compiles out — without the
 first line a handle to a destroyed world would reach whatever has since taken its slot.

 @param [&in] def : "the base embedded in the caller's definition"
 @return "void, or INVALID_DEFINITION when this world no longer exists, when the definition is unmarked, or when it names a body this world cannot resolve"
*>
fn void? check_joint_def(WorldId world_id, JointDef* def) @private {
    if (!world_id.is_valid()) return INVALID_DEFINITION~;
    if (def.internal_value != default_filter_joint_def().internal_value) return INVALID_DEFINITION~;
    if (!def.body_id_a.is_valid() || !def.body_id_b.is_valid()) return INVALID_DEFINITION~;

    int world0 = world_id.index1 - 1;
    if (def.body_id_a.world0 != world0 || def.body_id_b.world0 != world0) return INVALID_DEFINITION~;
}
