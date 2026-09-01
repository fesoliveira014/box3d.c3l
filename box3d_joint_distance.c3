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
 Narrows a joint to the handle its distance accessors take. box3d resolves each of those through a
 joint-type assertion this release build compiles out, so one reaching a joint of another type
 reinterprets that joint's solver union instead — a getter hands back nonsense and a setter writes
 over the other type's state. This conversion is where that is caught; an explicit
 (DistanceJointId) cast asserts the type rather than checking it, which is what an explicit cast
 means in C3. The handle is checked live before its type is read at all, because reading the type
 of a destroyed joint resolves through an assertion compiled out the same way and reports whatever
 has since taken the slot.

 @return "the distance handle, INVALID_HANDLE when the joint no longer exists, or WRONG_JOINT_TYPE when this joint is of another type"
*>
fn DistanceJointId? JointId.as_distance(self) {
    if (!self.is_valid()) return INVALID_HANDLE~;
    if (self.type() != JointType.DISTANCE) return WRONG_JOINT_TYPE~;
    return (DistanceJointId)self;
}
