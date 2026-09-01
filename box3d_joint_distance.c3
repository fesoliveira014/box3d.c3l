module b3;

<*
 Links two bodies by a segment held at the definition's length, with an optional spring, limit and
 motor, and rigid unless the compound condition DistanceJointDef describes holds. The definition
 must come from default_distance_joint_def() and must name two bodies of this world, the same
 guard on the same embedded base that create_filter_joint describes, stopping in the same place.

 @param [&in] def : "a definition obtained from default_distance_joint_def(), naming two bodies of this world"
 @return "the joint, or the faults create_filter_joint documents"
*>
fn DistanceJointId? WorldId.create_distance_joint(self, DistanceJointDef* def) {
    check_joint_def(self, &def.base)!;
    JointId joint = create_distance_joint_raw(self, def).checked()!;
    return (DistanceJointId)joint;
}

<*
 Narrows a joint to the handle its distance accessors take. Those accessors resolve through a
 joint-type assertion this release build compiles out, so one reaching a joint of another type
 reinterprets that joint's solver union — nonsense from a getter, an overwrite from a setter.
 Liveness is checked first: a destroyed joint's type resolves through the same compiled-out check.

 @return "the distance handle, INVALID_HANDLE when the joint no longer exists, or WRONG_JOINT_TYPE when this joint is of another type"
*>
fn DistanceJointId? JointId.as_distance(self) {
    if (!self.is_valid()) return INVALID_HANDLE~;
    if (self.type() != JointType.DISTANCE) return WRONG_JOINT_TYPE~;
    return (DistanceJointId)self;
}

<*
 The two forces this joint's spring is held between, which box3d reports through two
 out-parameters. The pair comes back the way set_spring_force_range stored it, inverted order
 included: that setter neither clamps nor swaps.

 @return "the joint's lower and upper spring force in newtons, -float::max and float::max on a joint built from a default definition"
*>
fn SpringForceRange DistanceJointId.spring_force_range(self) {
    SpringForceRange range;
    spring_force_range_raw(self, &range.lower, &range.upper);
    return range;
}
