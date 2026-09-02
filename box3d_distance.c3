module b3;

<*
 The closest points between two convex proxies, in shape A's frame.

 Every call starts GJK cold: box3d's warm-start cache belongs to one pair alone, so the wrapper
 keeps its own and zeroes it, and the diagnostic simplex array is never asked for — simplex_count
 comes back zero, and iterations counts the support points added past the first only where the
 cores stay apart, reading zero on any overlap whatever work the search did.

 @return "the closest points, or INVALID_DEFINITION when either proxy names no array, holds no points, or holds more than MAX_SHAPE_CAST_POINTS"
*>
fn DistanceOutput? shape_distance(DistanceInput input) {
    check_shape_proxy(input.proxy_a)!;
    check_shape_proxy(input.proxy_b)!;

    SimplexCache warm_start;
    return shape_distance_raw(&input, &warm_start, null, 0);
}

<*
 Sweeps proxy B along the translation at a fixed proxy A, in A's frame, reporting where they first
 touch. The cores close to the greater of one linear slop and the two radii less that slop, so the
 stop lands short of the geometric touch below a total radius of one slop, on it at one, and past
 above — a full slop past from two. A pair that begins overlapped reports fraction zero under a
 zero normal, a NaN once normalised, unless can_encroach is set.

 @return "where the two first touch, NO_HIT when they never do or the touch lies past max_fraction, or INVALID_DEFINITION when either proxy names no array, holds no points, or holds more than MAX_SHAPE_CAST_POINTS"
*>
fn CastOutput? shape_cast(ShapeCastPairInput input) {
    check_shape_proxy(input.proxy_a)!;
    check_shape_proxy(input.proxy_b)!;

    CastOutput result = shape_cast_pair_raw(&input);
    if (!result.hit) return NO_HIT~;
    return result;
}
