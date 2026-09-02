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

fn void the_query_types_match_the_c_layout() @test {
    assert(b3::QueryFilter::size == 32);
    assert(b3::QueryFilter::alignment == 8);
    assert(b3::RayResult::size == 64);
    assert(b3::RayResult::alignment == 8);
    assert(b3::ShapeProxy::size == 16);
    assert(b3::RayCastInput::size == 28);
    assert(b3::RayCastInput::alignment == 4);
    assert(b3::ShapeCastInput::size == 40);
    assert(b3::ShapeCastInput::alignment == 8);
    assert(b3::CastOutput::size == 48);
    assert(b3::WorldCastOutput::size == b3::CastOutput::size);
    assert(b3::BodyCastResult::size == 56);
    assert(b3::TreeStats::size == 8);
}

fn void the_contact_types_match_the_c_layout() @test {
    assert(b3::ContactData::size == 48);
    assert(b3::ContactData::alignment == 8);
    assert(b3::Manifold::size == 268);
    assert(b3::ManifoldPoint::size == 56);
    assert(b3::MAX_MANIFOLD_POINTS == 4);
    assert(b3::MAX_SHAPE_CAST_POINTS == 128);
}
