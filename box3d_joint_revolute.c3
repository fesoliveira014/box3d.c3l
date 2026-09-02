module b3;

<*
 Links two bodies with a hinge: their local frames are held together and left free to turn about
 the frame's z axis, with an optional spring, limit and motor. The definition must come from
 default_revolute_joint_def() and must name two bodies of this world, the same guard on the same
 embedded base that create_filter_joint describes, stopping in the same place.

 @param [&in] def : "a definition obtained from default_revolute_joint_def(), naming two bodies of this world"
 @return "the joint, or the faults create_filter_joint documents"
*>
fn RevoluteJointId? WorldId.create_revolute_joint(self, RevoluteJointDef* def) {
    check_joint_def(self, &def.base)!;
    JointId joint = create_revolute_joint_raw(self, def).checked()!;
    return (RevoluteJointId)joint;
}

<*
 Narrows a joint to the handle its revolute accessors take. Those accessors resolve through a
 joint-type assertion this release build compiles out, so one reaching a joint of another type
 reinterprets that joint's solver union — nonsense from a getter, an overwrite from a setter.
 Liveness is checked first: a destroyed joint's type resolves through the same compiled-out check.

 @return "the revolute handle, INVALID_HANDLE when the joint no longer exists, or WRONG_JOINT_TYPE when this joint is of another type"
*>
fn RevoluteJointId? JointId.as_revolute(self) {
    if (!self.is_valid()) return INVALID_HANDLE~;
    if (self.type() != JointType.REVOLUTE) return WRONG_JOINT_TYPE~;
    return (RevoluteJointId)self;
}
