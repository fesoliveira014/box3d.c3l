module b3;

<*
 Links two bodies with a velocity drive: body B is pushed towards a relative linear and angular
 velocity, and pulled towards the transform the two local frames describe by a linear and an
 angular spring. Each of the four channels is switched by its own ceiling rather than an enable
 flag, as MotorJointDef describes, so a definition straight from default_motor_joint_def() applies
 nothing at all. That default is also required, naming two bodies of this world — the same guard
 create_filter_joint describes, stopping in the same place.

 @param [&in] def : "a definition obtained from default_motor_joint_def(), naming two bodies of this world"
 @return "the joint, or the faults create_filter_joint documents"
*>
fn MotorJointId? WorldId.create_motor_joint(self, MotorJointDef* def) {
    check_joint_def(self, &def.base)!;
    JointId joint = create_motor_joint_raw(self, def).checked()!;
    return (MotorJointId)joint;
}

<*
 Narrows a joint to the handle its motor accessors take. Those accessors resolve through a
 joint-type assertion this release build compiles out, so one reaching a joint of another type
 reinterprets that joint's solver union — nonsense from a getter, an overwrite from a setter.
 Liveness is checked first: a destroyed joint's type resolves through the same compiled-out check.

 @return "the motor handle, INVALID_HANDLE when the joint no longer exists, or WRONG_JOINT_TYPE when this joint is of another type"
*>
fn MotorJointId? JointId.as_motor(self) {
    if (!self.is_valid()) return INVALID_HANDLE~;
    if (self.type() != JointType.MOTOR) return WRONG_JOINT_TYPE~;
    return (MotorJointId)self;
}
