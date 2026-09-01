module b3;

<*
 Links two bodies at the relative transform their local frames describe, with one spring over the
 translation and another over the rotation — and a zero hertz asks for a rigid channel rather than
 an absent spring, as WeldJointDef describes. The definition must come from
 default_weld_joint_def() and must name two bodies of this world, the same guard on the same
 embedded base that create_filter_joint describes, stopping in the same place.

 @param [&in] def : "a definition obtained from default_weld_joint_def(), naming two bodies of this world"
 @return "the joint, or the faults create_filter_joint documents"
*>
fn WeldJointId? WorldId.create_weld_joint(self, WeldJointDef* def) {
    check_joint_def(self, &def.base)!;
    JointId joint = create_weld_joint_raw(self, def).checked()!;
    return (WeldJointId)joint;
}

<*
 Narrows a joint to the handle its weld accessors take. box3d resolves each of those through a
 joint-type assertion this release build compiles out, so one reaching a joint of another type
 reinterprets that joint's solver union instead — a getter hands back nonsense and a setter writes
 over the other type's state. This conversion is where that is caught; an explicit (WeldJointId)
 cast asserts the type rather than checking it, which is what an explicit cast means in C3. The
 handle is checked live before its type is read at all, because reading the type of a destroyed
 joint resolves through an assertion compiled out the same way and reports whatever has since
 taken the slot.

 @return "the weld handle, INVALID_HANDLE when the joint no longer exists, or WRONG_JOINT_TYPE when this joint is of another type"
*>
fn WeldJointId? JointId.as_weld(self) {
    if (!self.is_valid()) return INVALID_HANDLE~;
    if (self.type() != JointType.WELD) return WRONG_JOINT_TYPE~;
    return (WeldJointId)self;
}
