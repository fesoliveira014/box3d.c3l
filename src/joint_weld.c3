module b3;

<*
 Links two bodies at the relative transform their local frames describe, with one spring over the
 translation and another over the rotation — and a zero hertz asks for a rigid channel rather than
 an absent spring, as WeldJointDef describes. The definition must come from
 default_weld_joint_def() and name two bodies of this world — the guard check_joint_def describes.

 @param [&in] def : `a definition from default_weld_joint_def(), naming two bodies of this
   world`
 @return `the joint`
 @return? check_joint_def!
 @return? INVALID_HANDLE : `the world is locked, mid step or inside a callback box3d runs during
   one`
*>
fn WeldJointId? WorldId.create_weld_joint(self, WeldJointDef* def) {
    if (def == null) return INVALID_DEFINITION~;
    check_joint_def(self, &def.base)!;
    JointId joint = create_weld_joint_raw(self, def).checked()!;
    return (WeldJointId)joint;
}

<*
 Narrows a joint to the handle its weld accessors take — see check_joint_type
 for the hazard the two guards close.

 @return `the weld handle`
 @return? check_joint_type!
*>
fn WeldJointId? JointId.as_weld(self) {
    check_joint_type(self, JointType.WELD)!;
    return (WeldJointId)self;
}
