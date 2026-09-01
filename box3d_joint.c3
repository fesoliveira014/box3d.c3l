module b3;

<*
 Links two bodies with a joint that constrains nothing, so that they stop colliding.

 The suppression is not the filter joint's own doing: b3ShouldBodiesCollide walks the shorter
 body's joint list and refuses the pair for any joint whose collide_connected is false and which
 links the two, whatever its type. A filter joint is simply the joint type with nothing else
 attached, so set_collide_connected(true) on it puts the collision back, and the effect is
 observable only while at least one of the two bodies is dynamic — two non-dynamic bodies never
 collide anyway. Being a joint, it also keeps the two bodies in one simulation island.

 The definition must come from default_filter_joint_def(). box3d marks a definition with an
 internal cookie there and checks it with an assertion, but this package links a release build
 where that assertion is compiled out — an unmarked definition would be accepted and would produce
 a joint whose force threshold is zero rather than float::max, reporting a joint event on every
 awake step. That cookie check is the only one this guard restores: box3d also asserts valid local
 frames and non-negative thresholds, those assertions are compiled out here too and none of them
 are restored. Unset body identifiers are not caught either — box3d accepts a definition naming no
 bodies in a release build without complaint. This is a known limit of the guard matching the body
 and shape constructors, not an oversight.

 @param [&in] def : "a definition obtained from default_filter_joint_def(), naming both bodies"
 @return "the joint, INVALID_DEFINITION if the definition did not come from default_filter_joint_def(), or INVALID_HANDLE if box3d returns a null identifier — which b3CreateFilterJoint does only when the target world is locked, mid b3World_Step or inside a callback box3d runs during one"
*>
fn JointId? WorldId.create_filter_joint(self, FilterJointDef* def) {
    if (def.internal_value != default_filter_joint_def().internal_value) return INVALID_DEFINITION~;
    return create_filter_joint_raw(self, def).checked();
}
