module b3;

<*
 Links two bodies with a spring that holds body B off body A along local frame B's
 y-axis, pushing along the definition's normal. A default definition pushes nothing. That default
 is also required, naming two bodies of this world — the guard check_joint_def describes.

 @param [&in] def : `a definition from default_pogo_joint_def(), naming two bodies of this world`
 @return `the joint`
 @return? check_joint_def!
 @return? INVALID_HANDLE : `the world is locked, mid step or inside a callback box3d runs during
   one`
*>
fn PogoJointId? create_pogo_joint(WorldId world, PogoJointDef* def) {
    if (def == null) return INVALID_DEFINITION~;
    check_joint_def(world, &def.base)!;
    JointId joint = create_pogo_joint_raw(world, def).checked()!;
    return (PogoJointId)joint;
}

<*
 Narrows a joint to the handle its pogo accessors take — see check_joint_type
 for the hazard the two guards close.

 @return `the pogo handle`
 @return? check_joint_type!
*>
fn PogoJointId? JointId.as_pogo(self) {
    check_joint_type(self, JointType.POGO)!;
    return (PogoJointId)self;
}
