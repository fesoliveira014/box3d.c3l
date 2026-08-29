module b3_test;

import b3;

fn void identifiers_match_the_c_layout() @test {
    assert(b3::WorldId::size == 4);
    assert(b3::WorldId::alignment == 2);
    assert(b3::BodyId::size == 8);
    assert(b3::ShapeId::size == 8);
    assert(b3::JointId::size == 8);
    assert(b3::ContactId::size == 12);
}

fn void a_zeroed_identifier_is_null() @test {
    b3::WorldId world;
    b3::BodyId body;
    assert(world.is_null());
    assert(body.is_null());
}
