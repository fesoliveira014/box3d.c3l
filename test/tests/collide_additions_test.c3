module b3_test;

import b3;

// A two-metre square, a point inside it, a point on its lower edge, and a second copy of the upper
// right corner well inside the weld distance carrying a lower original index than the first.
fn b3::Point2D[7] a_square_with_an_inside_an_edge_and_a_duplicate_point() {
    return {
        { .p = { 0.0, 0.0 }, .separation = -0.1, .original_index = 1 },
        { .p = { 2.0, 0.0 }, .separation = -0.2, .original_index = 2 },
        { .p = { 2.0, 2.0 }, .separation = -0.3, .original_index = 3 },
        { .p = { 0.0, 2.0 }, .separation = -0.4, .original_index = 4 },
        { .p = { 1.0, 1.0 }, .separation = -0.5, .original_index = 5 },
        { .p = { 1.0, 0.0 }, .separation = -0.6, .original_index = 6 },
        { .p = { 2.0005, 2.0 }, .separation = -0.7, .original_index = 0 },
    };
}

fn void a_2d_hull_keeps_the_corners_counter_clockwise_from_the_lowest_leftmost() @test {
    b3::Point2D[7] points = a_square_with_an_inside_an_edge_and_a_duplicate_point();
    b3::Point2D[14] buffer;

    b3::Point2D[] hull = b3::hull_2d(points[..], buffer[..])!!;

    assert(hull.len == 4, "the inside point and the one on an edge are not corners");
    assert(hull[0].original_index == 1 && hull[1].original_index == 2);
    assert(hull[2].original_index == 0, "the welded corner keeps the lower original index");
    assert(hull[2].p.x == 2.0005f && hull[2].separation == -0.7f,
        "and is that point whole, not the first one moved");
    assert(hull[3].original_index == 4);
    assert(points[0].original_index == 1 && points[1].original_index == 4,
        "the candidates were sorted by x then y in place");
}

fn void a_2d_hull_buffer_shorter_than_twice_the_points_is_refused() @test {
    b3::Point2D[7] points = a_square_with_an_inside_an_edge_and_a_duplicate_point();
    b3::Point2D[13] buffer;

    if (catch err = b3::hull_2d(points[..], buffer[..])) {
        assert(err == b3::BUFFER_TOO_SMALL);
        assert(points[0].original_index == 1 && points[6].original_index == 0,
            "and the candidates were never handed to box3d to sort");
        return;
    }
    assert(false, "box3d writes past the hull's length while it builds; the binding must refuse");
}

// A square whose edges each bulge out by a tenth at their middles: the midpoints span the least
// area, so they go first.
fn b3::Point2D[8] a_bulging_square() {
    return {
        { .p = { 0.0, 0.0 }, .original_index = 0 },
        { .p = { 1.0, -0.1 }, .original_index = 1 },
        { .p = { 2.0, 0.0 }, .original_index = 2 },
        { .p = { 2.1, 1.0 }, .original_index = 3 },
        { .p = { 2.0, 2.0 }, .original_index = 4 },
        { .p = { 1.0, 2.1 }, .original_index = 5 },
        { .p = { 0.0, 2.0 }, .original_index = 6 },
        { .p = { -0.1, 1.0 }, .original_index = 7 },
    };
}

fn void simplifying_a_2d_hull_drops_the_vertices_spanning_the_least_area() @test {
    b3::Point2D[8] hull = a_bulging_square();

    b3::Point2D[] simplified = b3::simplify_hull_2d(hull[..], 4);

    assert(simplified.len == 4);
    assert(simplified[0].original_index == 0 && simplified[1].original_index == 2
        && simplified[2].original_index == 4 && simplified[3].original_index == 6,
        "the four corners remain, in their order");
}

fn void simplifying_a_2d_hull_drops_a_vertex_flatter_than_the_tolerance_below_target() @test {
    b3::Point2D[4] hull = {
        { .p = { 0.0, 0.0 }, .original_index = 0 },
        { .p = { 1.0, -1.0e-6 }, .original_index = 1 },
        { .p = { 2.0, 0.0 }, .original_index = 2 },
        { .p = { 1.0, 1.0 }, .original_index = 3 },
    };

    b3::Point2D[] simplified = b3::simplify_hull_2d(hull[..], 8);

    assert(simplified.len == 3, "a target above the count keeps nothing box3d counts as flat");
    assert(simplified[1].original_index == 2);
}

fn void simplifying_a_2d_hull_never_goes_below_a_triangle() @test {
    b3::Point2D[8] hull = a_bulging_square();
    b3::Point2D[8] untouched = a_bulging_square();

    b3::Point2D[] to_none = b3::simplify_hull_2d(hull[..], 0);
    b3::Point2D[] two = b3::simplify_hull_2d(untouched[:2], 3);

    assert(to_none.len == 3, "a target under three acts as three");
    assert(two.len == 2, "and a hull already at or under three comes back as it was");
}
