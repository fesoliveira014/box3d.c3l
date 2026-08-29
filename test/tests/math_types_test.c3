module b3_test;

import b3;

fn void math_types_match_the_c_layout() @test {
    assert(b3::Vec3::size == 12);
    assert(b3::Pos::size == 12);
    assert(b3::Quat::size == 16);
    assert(b3::Matrix3::size == 36);
    assert(b3::Transform::size == 28);
    assert(b3::WorldTransform::size == 28);
    assert(b3::Aabb::size == 24);
    assert(b3::Plane::size == 16);
    assert(b3::CosSin::size == 8);
    assert(b3::SegmentDistanceResult::size == 32);
}

fn void a_vector_survives_the_round_trip() @test {
    b3::Vec3 origin = { 0.0, 0.0, 0.0 };
    b3::Vec3 far_end = { 10.0, 0.0, 0.0 };
    b3::Vec3 query = { 3.0, 4.0, 0.0 };

    b3::Vec3 closest = b3::point_to_segment_distance(origin, far_end, query);

    assert(closest.x == 3.0);
    assert(closest.y == 0.0);
    assert(closest.z == 0.0);
}

fn void a_struct_returned_by_value_survives() @test {
    b3::CosSin at_zero = b3::compute_cos_sin(0.0);
    assert(at_zero.cosine == 1.0);
    assert(at_zero.sine == 0.0);
}

fn void the_standard_library_operators_apply_to_bound_vectors() @test {
    b3::Vec3 a = { 1.0, 2.0, 3.0 };
    b3::Vec3 b = { 10.0, 20.0, 30.0 };
    b3::Vec3 sum = a + b;

    assert(sum.x == 11.0);
    assert(b3::is_valid_vec3(sum));
    assert(a.dot(b) == 140.0);
}
