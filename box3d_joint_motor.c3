module b3;

<*
 Links two bodies with a velocity drive: body B is pushed towards a relative linear and angular
 velocity, and pulled towards the transform the two local frames describe by a linear and an
 angular spring. Each of the four channels is switched by its own ceiling rather than an enable
 flag, as MotorJointDef describes, so a default definition applies nothing at all. That default is
 also required, naming two bodies of this world — the guard check_joint_def describes.

 @param [&in] def : `a definition from default_motor_joint_def(), naming two bodies of this
   world`
 @return `the joint`
 @return? check_joint_def!
 @return? INVALID_HANDLE : `the world is locked, mid step or inside a callback box3d runs during
   one`
*>
fn MotorJointId? WorldId.create_motor_joint(self, MotorJointDef* def) {
    if (def == null) return INVALID_DEFINITION~;
    check_joint_def(self, &def.base)!;
    JointId joint = create_motor_joint_raw(self, def).checked()!;
    return (MotorJointId)joint;
}

<*
 Narrows a joint to the handle its motor accessors take — see check_joint_type
 for the hazard the two guards close.

 @return `the motor handle`
 @return? check_joint_type!
*>
fn MotorJointId? JointId.as_motor(self) {
    check_joint_type(self, JointType.MOTOR)!;
    return (MotorJointId)self;
}
