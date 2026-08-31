module b3_test;

import b3;

fn void the_event_types_match_the_c_layout() @test {
    assert(b3::SensorBeginTouchEvent::size == 16);
    assert(b3::SensorEndTouchEvent::size == 16);
    assert(b3::SensorEvents::size == 24);
    assert(b3::SensorEvents::alignment == 8);
    assert(b3::ContactBeginTouchEvent::size == 28);
    assert(b3::ContactEndTouchEvent::size == 28);
    assert(b3::ContactHitEvent::size == 72);
    assert(b3::ContactHitEvent::alignment == 8);
    assert(b3::ContactEvents::size == 40);
    assert(b3::BodyMoveEvent::size == 48);
    assert(b3::BodyMoveEvent::alignment == 8);
    assert(b3::BodyEvents::size == 16);
    assert(b3::JointEvent::size == 16);
    assert(b3::JointEvents::size == 16);
}
