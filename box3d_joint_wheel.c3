module b3;

<*
 Links a chassis to a wheel: body B spins freely about local frame B's z axis while its suspension
 travels along local frame A's x axis and its steering turns about that same axis, each bounded
 only by whichever spring, motor and limit the definition enables. Nothing is corrected: box3d's
 check that the suspension limits arrive in order is compiled out and not restored, and the
 steering pair has no such check at all. The definition must come from default_wheel_joint_def()
 and name two bodies of this world — the guard check_joint_def describes.

 @param [&in] def : `a definition from default_wheel_joint_def(), naming two bodies of this
   world`
 @return `the joint`
 @return? check_joint_def!
 @return? INVALID_HANDLE : `the world is locked, mid step or inside a callback box3d runs during
   one`
*>
fn WheelJointId? WorldId.create_wheel_joint(self, WheelJointDef* def) {
    if (def == null) return INVALID_DEFINITION~;
    check_joint_def(self, &def.base)!;
    JointId joint = create_wheel_joint_raw(self, def).checked()!;
    return (WheelJointId)joint;
}

<*
 Narrows a joint to the handle its wheel accessors take — see check_joint_type
 for the hazard the two guards close.

 @return `the wheel handle`
 @return? check_joint_type!
*>
fn WheelJointId? JointId.as_wheel(self) {
    check_joint_type(self, JointType.WHEEL)!;
    return (WheelJointId)self;
}
