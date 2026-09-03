module b3;

<*
 Links two bodies by a segment held at the definition's length, with an optional spring, limit and
 motor, and rigid unless the compound condition DistanceJointDef describes holds. The definition
 must come from default_distance_joint_def() and name two bodies of this world — the guard
 check_joint_def describes.

 @param [&in] def : `a definition from default_distance_joint_def(), naming two bodies of this
   world`
 @return `the joint`
 @return? check_joint_def!
 @return? INVALID_HANDLE : `the world is locked, mid step or inside a callback box3d runs during
   one`
*>
fn DistanceJointId? WorldId.create_distance_joint(self, DistanceJointDef* def) {
    if (def == null) return INVALID_DEFINITION~;
    check_joint_def(self, &def.base)!;
    JointId joint = create_distance_joint_raw(self, def).checked()!;
    return (DistanceJointId)joint;
}

<*
 Narrows a joint to the handle its distance accessors take — see check_joint_type
 for the hazard the two guards close.

 @return `the distance handle`
 @return? check_joint_type!
*>
fn DistanceJointId? JointId.as_distance(self) {
    check_joint_type(self, JointType.DISTANCE)!;
    return (DistanceJointId)self;
}

<*
 The two forces this joint's spring is held between, which box3d reports through two
 out-parameters. The pair comes back the way set_spring_force_range stored it, inverted order
 included: that setter neither clamps nor swaps.

 @return `the lower and upper spring force in newtons, -float::max and float::max on a joint built
   from a default definition`
*>
fn SpringForceRange DistanceJointId.spring_force_range(self) {
    SpringForceRange range;
    spring_force_range_raw(self, &range.lower, &range.upper);
    return range;
}
