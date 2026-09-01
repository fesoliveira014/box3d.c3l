module b3;

<*
 Links two bodies with a velocity drive: body B is pushed towards a relative linear and angular
 velocity, and pulled towards the transform the two local frames describe by a linear and an
 angular spring. The joint carries no enable flags — each of the four channels is switched by its
 own ceiling, as MotorJointDef describes, so a definition straight from
 default_motor_joint_def() applies nothing at all. The definition must come from that default and
 must name two bodies of this world, the same guard on the same embedded base that
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
 Narrows a joint to the handle its motor accessors take. box3d resolves each of those through a
 joint-type assertion this release build compiles out, so one reaching a joint of another type
 reinterprets that joint's solver union instead — a getter hands back nonsense and a setter writes
 over the other type's state. This conversion is where that is caught; an explicit (MotorJointId)
 cast asserts the type rather than checking it, which is what an explicit cast means in C3. The
 handle is checked live before its type is read at all, because reading the type of a destroyed
 joint resolves through an assertion compiled out the same way and reports whatever has since
 taken the slot.

 @return "the motor handle, INVALID_HANDLE when the joint no longer exists, or WRONG_JOINT_TYPE when this joint is of another type"
*>
fn MotorJointId? JointId.as_motor(self) {
    if (!self.is_valid()) return INVALID_HANDLE~;
    if (self.type() != JointType.MOTOR) return WRONG_JOINT_TYPE~;
    return (MotorJointId)self;
}
