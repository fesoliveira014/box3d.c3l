module b3;

<*
 The contacts that began touching during the last step.

 The array belongs to the world and the next step() overwrites it, so anything that must outlive
 the step is copied out; destroying the world frees it outright. A shape reports contact events
 only if its definition asked for them, and enabling them on one of the two shapes is enough.

 @return "the begin-touch events from the last step, empty when nothing began touching"
*>
fn ContactBeginTouchEvent[] WorldId.contact_begin_touch_events(self) {
    ContactEvents events = contact_events_raw(self);
    return events.begin_events[:events.begin_count];
}

<*
 The contacts that stopped touching during the last step. Destroying or moving a body or shape,
 or changing a filter, also ends a contact, so an event here may name shapes and a contact that
 are already gone — ShapeId.is_valid() and ContactId.is_valid() are the checks. The array
 belongs to the world; the next step() overwrites it and destroying the world frees it.

 @return "the end-touch events from the last step, empty when nothing stopped touching"
*>
fn ContactEndTouchEvent[] WorldId.contact_end_touch_events(self) {
    ContactEvents events = contact_events_raw(self);
    return events.end_events[:events.end_count];
}

<*
 The collisions during the last step that closed faster than the world's hit event threshold. A
 collision is reported if either shape's definition set enable_hit_events — the other shape does
 not need to have asked. The array belongs to the world; the next step() overwrites it and
 destroying the world frees it.

 @return "the hit events from the last step, empty when nothing hit hard enough"
*>
fn ContactHitEvent[] WorldId.contact_hit_events(self) {
    ContactEvents events = contact_events_raw(self);
    return events.hit_events[:events.hit_count];
}

<*
 The shapes that began overlapping a sensor during the last step. Both shapes must have set
 enable_sensor_events in their definition — the visitor as well as the sensor — or the overlap
 goes unreported. The array belongs to the world; the next step() overwrites it and destroying
 the world frees it.

 @return "the sensor begin-touch events from the last step, empty when nothing entered a sensor"
*>
fn SensorBeginTouchEvent[] WorldId.sensor_begin_touch_events(self) {
    SensorEvents events = sensor_events_raw(self);
    return events.begin_events[:events.begin_count];
}

<*
 The shapes that stopped overlapping a sensor during the last step. Destroying or moving either
 shape also ends an overlap, so an event here may name a shape that is already gone —
 ShapeId.is_valid() is the check. The array belongs to the world; the next step() overwrites it
 and destroying the world frees it.

 @return "the sensor end-touch events from the last step, empty when nothing left a sensor"
*>
fn SensorEndTouchEvent[] WorldId.sensor_end_touch_events(self) {
    SensorEvents events = sensor_events_raw(self);
    return events.end_events[:events.end_count];
}

<*
 The bodies the simulation moved during the last step. A body the caller moved itself is not
 reported, and a sleeping body reports nothing until it wakes. Reading a pose from here is
 cheaper than calling BodyId.transform() per body, because only the bodies that moved appear and
 they arrive contiguously. The array belongs to the world; the next step() overwrites it and
 destroying the world frees it.

 @return "the move events from the last step, empty when nothing moved"
*>
fn BodyMoveEvent[] WorldId.body_move_events(self) {
    BodyEvents events = body_events_raw(self);
    return events.move_events[:events.move_count];
}

<*
 The awake joints whose force or torque exceeded their threshold during the last step. The
 forces themselves are not reported. A joint here may already have been destroyed —
 JointId.is_valid() is the check. The array belongs to the world; the next step() overwrites it
 and destroying the world frees it.

 @return "the joint events from the last step, empty when no joint exceeded its threshold"
*>
fn JointEvent[] WorldId.joint_events(self) {
    JointEvents events = joint_events_raw(self);
    return events.joint_events[:events.count];
}
