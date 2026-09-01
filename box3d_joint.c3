module b3;

<*
 Links two bodies with a joint that constrains nothing, so that they stop colliding.

 A filter joint is simply the joint type with nothing else attached, and the suppression is
 collide_connected's doing rather than the type's: b3ShouldBodiesCollide refuses the pair for any
 joint linking the two bodies whose flag is false, whatever its type, and set_collide_connected
 carries that mechanism in full. Being a joint, it also links the two bodies into one island, so
 that waking one wakes the other — but box3d adds a joint to the island graph only once at least
 one of the bodies is dynamic and neither is disabled, and a static body carries no island of its
 own, so against one the joint simply joins the dynamic body's.

 The definition must come from default_filter_joint_def() and must name two bodies of this world.
 This release build compiles box3d's own checks out, so an unmarked definition would be accepted
 and would carry a force threshold of zero rather than float::max, reporting a joint event on every
 awake step, while a body identifier this world cannot resolve — unset, stale, or belonging to
 another world — would be indexed and written through, since creating the joint writes that body's
 joint list head and count. Those three are the only checks restored: box3d also asserts valid
 local frames and non-negative finite thresholds, neither of those assertions is restored here, and
 that is a deliberate limit rather than an oversight — an unvalidated frame or threshold produces a
 wrong simulation, while an unvalidated identifier is dereferenced and written through.

 @param [&in] def : "a definition obtained from default_filter_joint_def(), naming two bodies of this world"
 @return "the joint, INVALID_DEFINITION if the definition did not come from default_filter_joint_def() or names a body this world cannot resolve, or INVALID_HANDLE if box3d returns a null identifier — which b3CreateFilterJoint does only when the target world is locked, mid b3World_Step or inside a callback box3d runs during one"
*>
fn JointId? WorldId.create_filter_joint(self, FilterJointDef* def) {
    check_joint_def(self, &def.base)!;
    return create_filter_joint_raw(self, def).checked();
}

<*
 The soft-constraint tuning this joint is solved with, which box3d reports through two
 out-parameters.

 @return "the joint's stiffness in hertz and its damping ratio, 60 and 2 on a joint built from a default definition"
*>
fn ConstraintTuning JointId.constraint_tuning(self) {
    ConstraintTuning tuning;
    constraint_tuning_raw(self, &tuning.hertz, &tuning.damping_ratio);
    return tuning;
}

<*
 Writes the joints linked to this body into the caller's buffer, most recently created first:
 box3d walks a list each new joint is inserted at the head of, so a body linked by one joint and
 then another reports the second before the first. joint_count() is exact, so a buffer that long
 is never truncated, and a buffer with no room at all takes nothing. Like joint_count(), the fill
 keeps working while the world is locked: box3d reaches the world here without the locked check,
 rather than filling nothing the way the sensor filler does.

 @return "the prefix of buffer that was filled"
*>
fn JointId[] BodyId.joints(self, JointId[] buffer) {
    int written = body_joints_raw(self, buffer.ptr, (int)buffer.len);
    return buffer[:written];
}

<*
 The one implementation of the checks every joint constructor must run. Every joint definition
 embeds the same base, and every default carries the same cookie, and box3d's own b3Body_IsValid
 answers for the world an identifier came from rather than the world the call was handed, so the
 two worlds are compared separately. See WorldId.create_filter_joint for what the three checks buy
 and where the guard deliberately stops.

 @param [&in] def : "the base embedded in the caller's definition"
 @return "void, or INVALID_DEFINITION when the definition is unmarked or names a body this world cannot resolve"
*>
fn void? check_joint_def(WorldId world_id, JointDef* def) @private {
    if (def.internal_value != default_filter_joint_def().internal_value) return INVALID_DEFINITION~;
    if (!def.body_id_a.is_valid() || !def.body_id_b.is_valid()) return INVALID_DEFINITION~;

    int world0 = world_id.index1 - 1;
    if (def.body_id_a.world0 != world0 || def.body_id_b.world0 != world0) return INVALID_DEFINITION~;
}
