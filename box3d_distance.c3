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
 Sweeps proxy B along the translation at a fixed proxy A, in A's frame. The cores close to one
 linear slop or the two radii less that slop, whichever is greater, so the stop is short of the
 geometric touch below a total radius of one slop, on it at one, past above, a full slop from
 two. A pair beginning overlapped answers fraction zero under a zero normal, a NaN once normalised,
 unless can_encroach is set. A max_fraction above 1 sweeps past the translation, so it is refused.

 @return "where the two first touch, NO_HIT when they never do or the touch lies past max_fraction, or INVALID_DEFINITION when either proxy names no array, holds no points, holds more than MAX_SHAPE_CAST_POINTS, or max_fraction is outside 0 to 1 or is not a number"
*>
fn CastOutput? shape_cast(ShapeCastPairInput input) {
    check_shape_proxy(input.proxy_a)!;
    check_shape_proxy(input.proxy_b)!;
    if (!(input.max_fraction >= 0.0f && input.max_fraction <= 1.0f)) return INVALID_DEFINITION~;

    CastOutput result = shape_cast_pair_raw(&input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Where this sweep places the shape's own origin at the given fraction of its interval.

 The rotation is a normalised lerp, not a slerp: slowest at the two ends, and equal to a slerp only
 there and at the midpoint. A fraction outside the interval extrapolates the centre linearly and
 the rotation at a falling rate, past an assertion this build compiles out, so it is refused here.

 @return "the transform at that fraction, or INVALID_DEFINITION when time is outside 0 to 1 or is not a number"
*>
fn Transform? Sweep.transform_at(&self, float time) {
    if (!(time >= 0.0f && time <= 1.0f)) return INVALID_DEFINITION~;
    return sweep_transform_raw(self, time);
}

<*
 Sweeps each proxy along its own sweep and reports where the two first touch, in the sweeps' frame.

 The cores close to the greater of one linear slop and the two radii less that slop, so the stop
 lands short of the geometric touch below a total radius of one slop, on it at one, and past above
 — a full slop past from two. A max_fraction outside the interval extrapolates both sweeps or a
 negative fraction, past an assertion this build compiles out, so it is refused here.

 @return "what the sweep found, under one of the four ToiState outcomes, or INVALID_DEFINITION when either proxy names no array, holds no points, or holds more than MAX_SHAPE_CAST_POINTS, or max_fraction is outside 0 to 1 or is not a number"
*>
fn ToiOutput? time_of_impact(ToiInput input) {
    check_shape_proxy(input.proxy_a)!;
    check_shape_proxy(input.proxy_b)!;
    if (!(input.max_fraction >= 0.0f && input.max_fraction <= 1.0f)) return INVALID_DEFINITION~;

    return time_of_impact_raw(&input);
}
