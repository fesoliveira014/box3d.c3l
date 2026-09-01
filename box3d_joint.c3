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

 The definition must come from default_filter_joint_def() and must name two bodies of this world,
 and this world must itself still exist. This release build compiles box3d's own checks out, so an
 unmarked definition would be accepted and would carry a force threshold of zero rather than
 float::max, reporting a joint event on every awake step; a body identifier this world cannot
 resolve — unset, stale, or belonging to another world — would be indexed and written through,
 since creating the joint writes that body's joint list head and count; and a handle to a destroyed
 world would reach whatever world has since taken its slot. Those four are the only checks
 restored: box3d also asserts valid local frames and non-negative finite thresholds, neither of
 those assertions is restored here, and that is a deliberate limit rather than an oversight — an
 unvalidated frame or threshold produces a wrong simulation, while an unvalidated identifier is
 dereferenced and written through.

 @param [&in] def : "a definition obtained from default_filter_joint_def(), naming two bodies of this world"
 @return "the joint, INVALID_DEFINITION if this world no longer exists, if the definition did not come from its type's default definition, or if it names a body this world cannot resolve, or INVALID_HANDLE if box3d returns a null identifier — which the type's constructor does only when the target world is locked, mid b3World_Step or inside a callback box3d runs during one"
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
 two worlds are compared separately. See WorldId.create_filter_joint for what the four checks buy
 and where the guard deliberately stops.

 The world is validated outright rather than taken for live because a body of it resolved, so each
 check stands on its own and reordering or dropping a line narrows what the guard catches instead
 of quietly reopening a hole another line was closing. The world check is not redundant either:
 box3d's generation test on a world identifier is an assertion this release build compiles out, so
 a handle to a destroyed world whose slot has since been re-allocated would otherwise reach the
 live world standing in its place.

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
