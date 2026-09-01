module b3;

<*
 Links a chassis to a wheel: body B spins freely about local frame B's z axis while its suspension
 travels along local frame A's x axis and its steering turns about that same axis, each of the
 three bounded only by whichever spring, motor and limit the definition enables. The definition
 must come from default_wheel_joint_def() and must name two bodies of this world, the same guard
 on the same embedded base that create_filter_joint describes, stopping in the same place. The
 constructor corrects nothing — every field is copied across verbatim — but box3d's own check that
 the suspension limits arrive in order is compiled out of this build and is not restored, and
 there is no such check on the steering pair at all.

 @param [&in] def : "a definition obtained from default_wheel_joint_def(), naming two bodies of this world"
 @return "the joint, or the faults create_filter_joint documents"
*>
fn WheelJointId? WorldId.create_wheel_joint(self, WheelJointDef* def) {
    check_joint_def(self, &def.base)!;
    JointId joint = create_wheel_joint_raw(self, def).checked()!;
    return (WheelJointId)joint;
}

<*
 Narrows a joint to the handle its wheel accessors take. box3d resolves each of those through a
 joint-type assertion this release build compiles out, so one reaching a joint of another type
 reinterprets that joint's solver union instead — a getter hands back nonsense and a setter writes
 over the other type's state. This conversion is where that is caught; an explicit (WheelJointId)
 cast asserts the type rather than checking it, which is what an explicit cast means in C3. The
 handle is checked live before its type is read at all, because reading the type of a destroyed
 joint resolves through an assertion compiled out the same way and reports whatever has since
 taken the slot.

 @return "the wheel handle, INVALID_HANDLE when the joint no longer exists, or WRONG_JOINT_TYPE when this joint is of another type"
*>
fn WheelJointId? JointId.as_wheel(self) {
    if (!self.is_valid()) return INVALID_HANDLE~;
    if (self.type() != JointType.WHEEL) return WRONG_JOINT_TYPE~;
    return (WheelJointId)self;
}
