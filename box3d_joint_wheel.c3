module b3;

<*
 Links a chassis to a wheel: body B spins freely about local frame B's z axis while its suspension
 travels along local frame A's x axis and its steering turns about that same axis, each bounded
 only by whichever spring, motor and limit the definition enables. The definition must come from
 default_wheel_joint_def() and name two bodies of this world, the guard create_filter_joint
 describes. Nothing is corrected: box3d's check that the suspension limits arrive in order is
 compiled out and not restored, and the steering pair has no such check at all.

 @param [&in] def : "a definition obtained from default_wheel_joint_def(), naming two bodies of this world"
 @return "the joint, or the faults create_filter_joint documents"
*>
fn WheelJointId? WorldId.create_wheel_joint(self, WheelJointDef* def) {
    check_joint_def(self, &def.base)!;
    JointId joint = create_wheel_joint_raw(self, def).checked()!;
    return (WheelJointId)joint;
}

<*
 Narrows a joint to the handle its wheel accessors take. Those accessors resolve through a
 joint-type assertion this release build compiles out, so one reaching a joint of another type
 reinterprets that joint's solver union — nonsense from a getter, an overwrite from a setter.
 Liveness is checked first: a destroyed joint's type resolves through the same compiled-out check.

 @return "the wheel handle, INVALID_HANDLE when the joint no longer exists, or WRONG_JOINT_TYPE when this joint is of another type"
*>
fn WheelJointId? JointId.as_wheel(self) {
    if (!self.is_valid()) return INVALID_HANDLE~;
    if (self.type() != JointType.WHEEL) return WRONG_JOINT_TYPE~;
    return (WheelJointId)self;
}
