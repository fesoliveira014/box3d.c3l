module b3_test;

import b3;
import std::math;

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

fn void segment_distance_reports_each_field_of_two_skew_segments() @test {
    b3::Vec3 p1 = { 0.0, 0.0, 0.0 };
    b3::Vec3 q1 = { 6.0, 0.0, 0.0 };
    b3::Vec3 p2 = { 1.0, -3.0, 1.0 };
    b3::Vec3 q2 = { 1.0, 9.0, 1.0 };

    b3::SegmentDistanceResult result = b3::segment_distance(
      p1: p1,
      q1: q1,
      p2: p2,
      q2: q2,
    );

    assert(result.point1.x == 1.0);
    assert(result.point1.y == 0.0);
    assert(result.point1.z == 0.0);
    assert(result.fraction1 == 0.16666667f);
    assert(result.point2.x == 1.0);
    assert(result.point2.y == 0.0);
    assert(result.point2.z == 1.0);
    assert(result.fraction2 == 0.25);
}

fn void is_valid_aabb_requires_upper_at_or_above_lower() @test {
    b3::Aabb ordered = { .lower_bound = { -1.0, -1.0, -1.0 }, .upper_bound = { 1.0, 1.0, 1.0 } };
    b3::Aabb inverted = { .lower_bound = { 1.0, 1.0, 1.0 }, .upper_bound = { -1.0, -1.0, -1.0 } };

    assert(b3::is_valid_aabb(ordered));
    assert(!b3::is_valid_aabb(inverted));
}

fn void is_valid_plane_accepts_a_unit_normal_and_a_nonzero_offset() @test {
    b3::Plane plane = { .normal = { 0.0, 1.0, 0.0 }, .offset = 3.5 };

    assert(b3::is_valid_plane(plane));
}

fn void is_valid_transform_accepts_a_nonsymmetric_position_and_identity_rotation() @test {
    b3::Transform xf = { .p = { 1.0, 2.0, 3.0 }, .q = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 } };

    assert(b3::is_valid_transform(xf));
}

fn void is_valid_quat_separates_a_unit_rotation_from_a_zero_one() @test {
    b3::Quat identity = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 };
    b3::Quat zeroed;

    assert(b3::is_valid_quat(identity));
    assert(!b3::is_valid_quat(zeroed), "a zero quaternion has no rotation to represent");
}

fn void the_three_aabb_predicates_are_independent_of_one_another() @test {
    b3::Aabb ordered = { .lower_bound = { -1.0, -1.0, -1.0 }, .upper_bound = { 1.0, 2.0, 3.0 } };
    b3::Aabb inverted = { .lower_bound = { 1.0, 1.0, 1.0 }, .upper_bound = { -1.0, -1.0, -1.0 } };
    b3::Aabb far_out = { .lower_bound = { -1.0e9, -1.0, -1.0 }, .upper_bound = { 1.0e9, 1.0, 1.0 } };

    assert(b3::is_valid_aabb(ordered) && b3::is_bounded_aabb(ordered) && b3::is_sane_aabb(ordered),
        "an ordered box near the origin passes all three");

    assert(!b3::is_valid_aabb(inverted), "an upper bound under the lower is not valid");
    assert(b3::is_bounded_aabb(inverted), "and bounded says nothing at all about the ordering");
    assert(!b3::is_sane_aabb(inverted), "so sane, which is both, fails on the ordering alone");

    assert(b3::is_valid_aabb(far_out), "a box a billion units across is ordered and finite");
    assert(!b3::is_bounded_aabb(far_out), "and out past the 1e5 length units box3d calls huge");
    assert(!b3::is_sane_aabb(far_out), "so sane fails on the magnitude alone");
}

fn void a_valid_matrix_is_only_a_finite_one() @test {
    b3::Matrix3 identity = {
      .cx = { 1.0, 0.0, 0.0 },
      .cy = { 0.0, 1.0, 0.0 },
      .cz = { 0.0, 0.0, 1.0 },
    };
    b3::Matrix3 zeroed;
    float infinity = (float)1.0e40;
    b3::Matrix3 with_a_nan = identity;
    with_a_nan.cy.z = infinity - infinity;

    assert(b3::is_valid_matrix3(identity));
    assert(b3::is_valid_matrix3(zeroed), "a matrix that is no rotation at all is still valid");
    assert(!b3::is_valid_matrix3(with_a_nan), "where one entry that is not a number is not");
}

fn void a_world_transform_bounds_its_rotation_and_not_its_position() @test {
    b3::WorldTransform far_out = {
      .p = { 1.0e30, 0.0, 0.0 },
      .q = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 },
    };
    b3::WorldTransform slack_rotation = {
      .p = { 1.0, 2.0, 3.0 },
      .q = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 2.0 },
    };

    assert(b3::is_valid_world_transform(far_out), "nothing bounds how far out a world position is");
    assert(!b3::is_valid_world_transform(slack_rotation), "but the rotation must be a unit quaternion");
}

fn void atan2_tracks_the_true_arc_tangent_over_a_full_turn() @test {
    double worst = 0.0;
    for (int tenths = -1800; tenths <= 1800; tenths++) {
        double radians = (double)tenths * 0.1 * math::PI / 180.0;
        float y = (float)math::sin(radians);
        float x = (float)math::cos(radians);
        double error = math::abs((double)b3::atan2(y, x) - math::atan2((double)y, (double)x));
        if (error > worst) worst = error;
    }

    assert(worst > 1.0e-6, "the approximation is not exact, so a worst error of nothing means nothing was sampled");
    assert(worst < 3.0e-5, "and it stays inside 0.0000277 radians, tighter than the 0.0023 degrees box3d claims");
}

fn void atan2_answers_zero_for_the_origin_alone() @test {
    float straight_up = b3::atan2(1.0e-30f, 0.0);

    assert(straight_up > 1.57 && straight_up < 1.58, "a vanishing y over a zero x is still a quarter turn");
    assert(b3::atan2(0.0, 0.0) == 0.0, "where both at zero is box3d's own case, added to match the standard library");
}

fn void steiner_matches_the_parallel_axis_theorem() @test {
    b3::Matrix3 tensor = b3::steiner(3.0, { 1.0, 2.0, 3.0 });

    assert(tensor.cx.x == 39.0 && tensor.cy.y == 30.0 && tensor.cz.z == 15.0,
        "the mass times the squared distance across each axis, down the diagonal");
    assert(tensor.cy.x == -6.0 && tensor.cz.x == -9.0 && tensor.cz.y == -18.0,
        "and the negated mass-weighted products off it");
    assert(tensor.cx.y == tensor.cy.x && tensor.cx.z == tensor.cz.x && tensor.cy.z == tensor.cz.y,
        "which is symmetric, so no column order can be read off it");
}

fn void steiner_puts_nothing_on_the_axis_the_offset_runs_along() @test {
    b3::Matrix3 tensor = b3::steiner(2.0, { 3.0, 0.0, 0.0 });

    assert(tensor.cy.y == 18.0 && tensor.cz.z == 18.0, "mass times the squared offset across the other two axes");
    assert(tensor.cx.x == 0.0, "and nothing at all along the offset itself");
}

fn void a_quaternion_from_a_matrix_reads_columns_and_not_rows() @test {
    b3::Matrix3 quarter_turn_about_z = {
      .cx = { 0.0, 1.0, 0.0 },
      .cy = { -1.0, 0.0, 0.0 },
      .cz = { 0.0, 0.0, 1.0 },
    };
    b3::Matrix3 transposed = {
      .cx = { 0.0, -1.0, 0.0 },
      .cy = { 1.0, 0.0, 0.0 },
      .cz = { 0.0, 0.0, 1.0 },
    };

    b3::Quat turn = b3::make_quat_from_matrix(quarter_turn_about_z);
    b3::Quat the_other_way = b3::make_quat_from_matrix(transposed);
    b3::Vec3 turned = turn.rotate_vec3({ 1.0, 0.0, 0.0 });

    assert(turned.y > 0.99 && turned.y < 1.01, "the columns turn x a quarter of the way onto y");
    assert(turn.k > 0.706 && turn.k < 0.708, "which is a quarter turn about positive z");
    assert(the_other_way.k > -0.708 && the_other_way.k < -0.706,
        "and the same matrix filled from rows is the same turn the other way, which no layout pin can catch");
}

fn void a_rotation_between_antipodal_vectors_is_a_half_turn() @test {
    b3::Vec3 forward = { 1.0, 0.0, 0.0 };
    b3::Vec3 backward = { -1.0, 0.0, 0.0 };

    b3::Quat half_turn = b3::compute_quat_between_unit_vectors(forward, backward)!!;
    b3::Vec3 turned = half_turn.rotate_vec3(forward);

    assert(turned.x > -1.01 && turned.x < -0.99, "the half turn takes x onto its opposite");
    assert(half_turn.j > -1.01 && half_turn.j < -0.99, "about the axis box3d picks perpendicular to it");
}

fn void a_rotation_between_vectors_refuses_a_pair_that_is_not_unit() @test {
    b3::Vec3 unit_x = { 1.0, 0.0, 0.0 };
    b3::Vec3 three_along_y = { 0.0, 3.0, 0.0 };
    b3::Vec3 unit_y = { 0.0, 1.0, 0.0 };

    fault refusal;
    if (catch err = b3::compute_quat_between_unit_vectors(unit_x, three_along_y)) refusal = err;
    b3::Quat between_unit_vectors = b3::compute_quat_between_unit_vectors(unit_x, unit_y)!!;

    assert(refusal == b3::NOT_A_UNIT_VECTOR,
        "a vector three units long reads as a 143 degree turn where these two are a quarter apart");
    assert(between_unit_vectors.k > 0.706 && between_unit_vectors.k < 0.708,
        "and the unit pair the guard admits answers that quarter turn");
}

fn void line_distance_takes_directions_where_segment_distance_takes_endpoints() @test {
    b3::Vec3 origin = { 0.0, 0.0, 0.0 };
    b3::Vec3 far_end = { 6.0, 0.0, 0.0 };
    b3::Vec3 side_start = { 1.0, -3.0, 1.0 };
    b3::Vec3 side_end = { 1.0, 9.0, 1.0 };

    b3::SegmentDistanceResult as_segments = b3::segment_distance(
      p1: origin,
      q1: far_end,
      p2: side_start,
      q2: side_end,
    );
    b3::SegmentDistanceResult the_same_four = b3::line_distance(
      p1: origin,
      d1: far_end,
      p2: side_start,
      d2: side_end,
    )!!;
    b3::SegmentDistanceResult as_directions = b3::line_distance(
      p1: origin,
      d1: far_end - origin,
      p2: side_start,
      d2: side_end - side_start,
    )!!;

    assert(the_same_four.fraction1 > 0.219 && the_same_four.fraction1 < 0.220,
        "read as directions the four vectors name lines that meet elsewhere");
    assert(as_segments.fraction1 > 0.166 && as_segments.fraction1 < 0.167,
        "where read as endpoints they name segments that meet a sixth of the way along");
    assert(as_directions.fraction1 == as_segments.fraction1 && as_directions.fraction2 == as_segments.fraction2,
        "and the endpoints turned into directions put the two back into agreement");
}

fn void a_line_reports_a_closest_point_behind_its_own_point() @test {
    b3::Vec3 origin = { 0.0, 0.0, 0.0 };
    b3::Vec3 along_x = { 1.0, 0.0, 0.0 };
    b3::Vec3 up_the_side = { 5.0, 2.0, 0.0 };
    b3::Vec3 along_y = { 0.0, 1.0, 0.0 };

    b3::SegmentDistanceResult crossing = b3::line_distance(
      p1: origin,
      d1: along_x,
      p2: up_the_side,
      d2: along_y,
    )!!;

    assert(crossing.fraction2 == -2.0, "two units back along the second line, which a segment could never report");
    assert(crossing.point1.x == 5.0 && crossing.point2.x == 5.0, "the two lines crossing at five along the first");
}

fn void line_distance_refuses_a_first_direction_of_no_length_and_answers_a_second() @test {
    b3::Vec3 origin = { 0.0, 0.0, 0.0 };
    b3::Vec3 along_x = { 1.0, 0.0, 0.0 };
    b3::Vec3 up_the_side = { 5.0, 2.0, 0.0 };
    b3::Vec3 nothing = { 0.0, 0.0, 0.0 };

    fault refusal;
    if (catch err = b3::line_distance(
      p1: origin,
      d1: nothing,
      p2: up_the_side,
      d2: along_x,
    )) refusal = err;
    b3::SegmentDistanceResult against_a_point = b3::line_distance(
      p1: origin,
      d1: along_x,
      p2: up_the_side,
      d2: nothing,
    )!!;

    assert(refusal == b3::DEGENERATE_GEOMETRY,
        "box3d divides by the first direction where the lines are parallel and answers a point of NaNs");
    assert(against_a_point.point2.y == 2.0 && against_a_point.point1.x == 5.0,
        "where a second direction of no length names a point, and box3d answers the place on the first line nearest it");
}
