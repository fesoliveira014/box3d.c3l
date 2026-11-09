module b3_test;

import b3;

fn void a_complex_hull_keeps_all_thirty_two_points_on_its_sphere() @test {
    b3::HullData* hull = b3::create_complex_hull(1.0)!!;
    defer b3::destroy_hull(hull);
    b3::HullData* again = b3::create_complex_hull(1.0)!!;
    defer b3::destroy_hull(again);
    b3::HullData* doubled = b3::create_complex_hull(2.0)!!;
    defer b3::destroy_hull(doubled);

    assert(hull.vertex_count == 32, "every point on a sphere is a vertex of their hull");
    assert(hull.volume > 2.9 && hull.volume < 2.95,
        "short of the unit ball's 4.19 by the gaps between thirty-two points");
    assert(hull.aabb.lower_bound.x > -1.0 && hull.aabb.upper_bound.x < 1.0,
        "and no further out than the radius");
    assert(again.hash == hull.hash, "the seed is fixed, so one radius always builds one hull");
    assert(doubled.volume > 8.0f * 2.9f && doubled.volume < 8.0f * 2.95f, "the radius scales it");
}

fn void a_complex_hull_too_small_to_separate_its_points_is_refused() @test {
    HullRefusal none = @the_refusal_of(b3::create_complex_hull(0.0));

    assert(none.refusal == b3::DEGENERATE_GEOMETRY, "thirty-two points at the origin are one point");
    assert(none.after == none.baseline);
}
