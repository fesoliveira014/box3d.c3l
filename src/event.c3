module b3;

<*
 The contacts that began touching during the last step.

 The array belongs to the world and the next step() overwrites it, so anything that must outlive
 the step is copied out; destroying the world frees it outright. A shape reports contact events
 only if its definition asked for them, and enabling them on one of the two shapes is enough.
*>
fn ContactBeginTouchEvent[] WorldId.contact_begin_touch_events(self) {
    ContactEvents events = contact_events_raw(self);
    return events.begin_events[:events.begin_count];
}

<*
 The contacts that stopped touching during the last step; the array is the world's, as in
 contact_begin_touch_events. Destroying or moving a body or shape, or changing a filter, also ends
 a contact, so an event may name shapes and a contact already gone — is_valid() is the check.
*>
fn ContactEndTouchEvent[] WorldId.contact_end_touch_events(self) {
    ContactEvents events = contact_events_raw(self);
    return events.end_events[:events.end_count];
}

<*
 The collisions during the last step that closed faster than the world's hit event threshold; the
 array is the world's, as in contact_begin_touch_events. Either shape's definition setting
 enable_hit_events is enough — the other need not have asked.
*>
fn ContactHitEvent[] WorldId.contact_hit_events(self) {
    ContactEvents events = contact_events_raw(self);
    return events.hit_events[:events.hit_count];
}

<*
 The shapes that began overlapping a sensor during the last step; the array is the world's, as in
 contact_begin_touch_events. Both shapes must have set enable_sensor_events — the visitor as well
 as the sensor — or the overlap goes unreported.
*>
fn SensorBeginTouchEvent[] WorldId.sensor_begin_touch_events(self) {
    SensorEvents events = sensor_events_raw(self);
    return events.begin_events[:events.begin_count];
}

<*
 The shapes that stopped overlapping a sensor during the last step; the array is the world's, as
 in contact_begin_touch_events. Destroying or moving either shape also ends an overlap, so an
 event may name a shape already gone — ShapeId.is_valid() is the check.
*>
fn SensorEndTouchEvent[] WorldId.sensor_end_touch_events(self) {
    SensorEvents events = sensor_events_raw(self);
    return events.end_events[:events.end_count];
}

<*
 The bodies the simulation moved during the last step; the array is the world's, as in
 contact_begin_touch_events. A body the caller moved itself is not reported, and a sleeping body
 reports nothing until it wakes.
*>
fn BodyMoveEvent[] WorldId.body_move_events(self) {
    BodyEvents events = body_events_raw(self);
    return events.move_events[:events.move_count];
}

<*
 The awake joints whose force or torque exceeded their threshold during the last step; the array
 is the world's, as in contact_begin_touch_events. The forces themselves are not reported, and a
 joint here may already have been destroyed — JointId.is_valid() is the check.
*>
fn JointEvent[] WorldId.joint_events(self) {
    JointEvents events = joint_events_raw(self);
    return events.joint_events[:events.count];
}
