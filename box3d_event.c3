module b3;

<*
 The contacts that began touching during the last step.

 The array belongs to the world and the next step() overwrites it, so anything that must outlive
 the step is copied out. A shape reports contact events only if its definition asked for them,
 and enabling them on one of the two shapes is enough.

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
 belongs to the world and the next step() overwrites it.

 @return "the end-touch events from the last step, empty when nothing stopped touching"
*>
fn ContactEndTouchEvent[] WorldId.contact_end_touch_events(self) {
    ContactEvents events = contact_events_raw(self);
    return events.end_events[:events.end_count];
}

<*
 The collisions during the last step that closed faster than the world's hit event threshold.
 Only shapes whose definition set enable_hit_events are reported. The array belongs to the world
 and the next step() overwrites it.

 @return "the hit events from the last step, empty when nothing hit hard enough"
*>
fn ContactHitEvent[] WorldId.contact_hit_events(self) {
    ContactEvents events = contact_events_raw(self);
    return events.hit_events[:events.hit_count];
}
