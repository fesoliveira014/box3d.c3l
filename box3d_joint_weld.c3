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
 Narrows a joint to the handle its weld accessors take. Those accessors resolve through a
 joint-type assertion this release build compiles out, so one reaching a joint of another type
 reinterprets that joint's solver union — nonsense from a getter, an overwrite from a setter.
 Liveness is checked first: a destroyed joint's type resolves through the same compiled-out check.

 @return "the weld handle, INVALID_HANDLE when the joint no longer exists, or WRONG_JOINT_TYPE when this joint is of another type"
*>
fn WeldJointId? JointId.as_weld(self) {
    if (!self.is_valid()) return INVALID_HANDLE~;
    if (self.type() != JointType.WELD) return WRONG_JOINT_TYPE~;
    return (WeldJointId)self;
}
