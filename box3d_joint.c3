module b3;

<*
 Links two bodies with a joint that constrains nothing, so that they stop colliding.

 The suppression is not the filter joint's own doing: b3ShouldBodiesCollide walks the shorter
 body's joint list and refuses the pair for any joint whose collide_connected is false and which
 links the two, whatever its type. A filter joint is simply the joint type with nothing else
 attached, so set_collide_connected(true) on it puts the collision back, and the effect is
 observable only while at least one of the two bodies is dynamic — two non-dynamic bodies never
 collide anyway. Being a joint, it also links the two bodies into one island, so that waking one
 wakes the other — but box3d adds a joint to the island graph only once at least one of the bodies
 is dynamic and neither is disabled, and a static body carries no island of its own, so against one
 the joint simply joins the dynamic body's.

 The definition must come from default_filter_joint_def() and must name both bodies. box3d marks a
 definition with an internal cookie there and checks it with an assertion, but this package links a
 release build where that assertion is compiled out — an unmarked definition would be accepted and
 would produce a joint whose force threshold is zero rather than float::max, reporting a joint event
 on every awake step. The body identifiers are checked because box3d's bounds assertion on them is
 compiled out too: an unset identifier indexes one slot before the start of the world's body array,
 and creating the joint writes the body's joint list head and count through it. Those two are the
 only checks this guard restores. box3d also asserts valid local frames and non-negative finite
 thresholds, those assertions are compiled out here as well and neither is restored, which is a
 known limit of the guard rather than an oversight — an unvalidated frame or threshold produces a
 wrong simulation, while an unvalidated identifier is dereferenced and written through.

 @param [&in] def : "a definition obtained from default_filter_joint_def(), naming both bodies"
 @return "the joint, INVALID_DEFINITION if the definition did not come from default_filter_joint_def() or leaves either body identifier unset, or INVALID_HANDLE if box3d returns a null identifier — which b3CreateFilterJoint does only when the target world is locked, mid b3World_Step or inside a callback box3d runs during one"
*>
fn JointId? WorldId.create_filter_joint(self, FilterJointDef* def) {
    if (def.internal_value != default_filter_joint_def().internal_value) return INVALID_DEFINITION~;
    if (def.body_id_a.is_null() || def.body_id_b.is_null()) return INVALID_DEFINITION~;
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
 is never truncated, and a buffer with no room at all takes nothing.

 @return "the prefix of buffer that was filled"
*>
fn JointId[] BodyId.joints(self, JointId[] buffer) {
    int written = body_joints_raw(self, buffer.ptr, (int)buffer.len);
    return buffer[:written];
}
