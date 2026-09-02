module b3_test;

import b3;
import std::math;

// What one distance call answered, flattened so a fixture can hand back several readings at once.
// answered is whether the wrapper returned a value rather than a fault, which is the whole of what
// a caller sees of a refusal.
struct DistanceReading {
    bool answered;
    float distance;
    b3::Vec3 normal;
    b3::Vec3 point_a;
    b3::Vec3 point_b;
    int iterations;
    bool reported_no_simplexes;
}

// An expression macro for the reason @the_reading_of is one: C3 forbids an optional parameter, so
// the call has to arrive unevaluated to keep a refusal from propagating out of the fixture.
macro DistanceReading @the_distance_of(#call) {
    if (try out = #call) {
        return {
          .answered = true,
          .distance = out.distance,
          .normal = out.normal,
          .point_a = out.point_a,
          .point_b = out.point_b,
          .iterations = out.iterations,
          .reported_no_simplexes = out.simplex_count == 0,
        };
    }
    return {};
}

// The proxies are placed by an offset along x, so a pair of single points has a core distance of
// that offset and a surface distance of it less the two radii. The points are borrowed for the
// length of the call, so each fixture holds its own arrays across it.
fn b3::DistanceInput a_point_pair(
  b3::Vec3[] a,
  float radius_a,
  b3::Vec3[] b,
  float radius_b,
  float offset,
  bool use_radii,
) {
    return {
      .proxy_a = { .points = a.ptr, .count = (int)a.len, .radius = radius_a },
      .proxy_b = { .points = b.ptr, .count = (int)b.len, .radius = radius_b },
      .transform = a_placement_at({ offset, 0.0, 0.0 }),
      .use_radii = use_radii,
    };
}

fn b3::ShapeCastPairInput a_swept_point_pair(
  b3::Vec3[] a,
  float radius_a,
  b3::Vec3[] b,
  float radius_b,
  float offset,
  float travel,
  float max_fraction,
  bool can_encroach,
) {
    return {
      .proxy_a = { .points = a.ptr, .count = (int)a.len, .radius = radius_a },
      .proxy_b = { .points = b.ptr, .count = (int)b.len, .radius = radius_b },
      .transform = a_placement_at({ offset, 0.0, 0.0 }),
      .translation_b = { travel, 0.0, 0.0 },
      .max_fraction = max_fraction,
      .can_encroach = can_encroach,
    };
}

struct PointPairDistances {
    DistanceReading cores;
    DistanceReading surfaces;
    DistanceReading uneven_surfaces;
}

// Three metres apart, with use_radii the only thing that changes between the first two. The third
// splits the same total radius unevenly, so a wrapper subtracting one radius rather than both
// answers it differently from the second.
fn PointPairDistances the_distances_between_two_points_three_apart() {
    b3::Vec3[1] a = { { 0.0, 0.0, 0.0 } };
    b3::Vec3[1] b = { { 0.0, 0.0, 0.0 } };

    b3::DistanceInput cores = a_point_pair(
      a: a[..],
      radius_a: 0.5,
      b: b[..],
      radius_b: 0.5,
      offset: 3.0,
      use_radii: false,
    );
    b3::DistanceInput surfaces = cores;
    surfaces.use_radii = true;

    b3::DistanceInput uneven = a_point_pair(
      a: a[..],
      radius_a: 0.25,
      b: b[..],
      radius_b: 0.75,
      offset: 3.0,
      use_radii: true,
    );

    return {
      .cores = @the_distance_of(b3::shape_distance(cores)),
      .surfaces = @the_distance_of(b3::shape_distance(surfaces)),
      .uneven_surfaces = @the_distance_of(b3::shape_distance(uneven)),
    };
}

fn void a_distance_query_subtracts_both_radii_only_when_it_is_asked_to() @test {
    PointPairDistances points = the_distances_between_two_points_three_apart();

    assert(points.cores.answered && points.surfaces.answered && points.uneven_surfaces.answered,
        "all three pairs are ordinary proxies and answer");

    assert(2.9999 < points.cores.distance && points.cores.distance < 3.0001,
        "without use_radii the answer is the gap between the two cores");
    assert(1.9999 < points.surfaces.distance && points.surfaces.distance < 2.0001,
        "with it the gap comes back less both radii, not less one of them");
    assert(math::abs(points.uneven_surfaces.distance - points.surfaces.distance) < 0.00001,
        "so the same total radius split unevenly answers the same");
    assert(math::abs(points.uneven_surfaces.point_a.x - 0.25f) < 0.00001
        && math::abs(points.uneven_surfaces.point_b.x - 2.25f) < 0.00001,
        "though its two witness points move, each out by the radius its own proxy carries");

    assert(math::abs(points.cores.point_a.x) < 0.00001 && math::abs(points.cores.point_b.x - 3.0f) < 0.00001,
        "the core witness points are the two proxy points themselves");
    assert(math::abs(points.surfaces.point_a.x - 0.5f) < 0.00001
        && math::abs(points.surfaces.point_b.x - 2.5f) < 0.00001,
        "and use_radii moves each of them out onto its own perimeter");

    assert(math::abs(points.surfaces.normal.x - 1.0f) < 0.00001
        && math::abs(the_length_of(points.surfaces.normal) - 1.0f) < 0.00001,
        "the normal runs from A towards B and is a unit vector");
}

struct BoxPairDiagnostics {
    DistanceReading clear;
    DistanceReading overlapping;
}

// The eight corners of a box a quarter metre out on every axis, small enough to sit wholly inside
// the two metre box the suite builds its hulls from.
fn b3::Vec3[8] the_corners_of_a_half_metre_box() {
    return {
        { -0.25, -0.25, -0.25 }, { 0.25, -0.25, -0.25 },
        { 0.25, 0.25, -0.25 }, { -0.25, 0.25, -0.25 },
        { -0.25, -0.25, 0.25 }, { 0.25, -0.25, 0.25 },
        { 0.25, 0.25, 0.25 }, { -0.25, 0.25, 0.25 },
    };
}

// Eight corners apiece at two different sizes, so each witness point lands somewhere its own box
// alone puts it and a query reading the two proxies the wrong way round answers differently. The
// second offset buries the smaller box in the larger, which box3d answers before counting at all.
fn BoxPairDiagnostics the_diagnostics_of_two_boxes() {
    b3::Vec3[8] a = the_corners_of_a_two_metre_box();
    b3::Vec3[8] b = the_corners_of_a_half_metre_box();

    b3::DistanceInput clear = a_point_pair(
      a: a[..],
      radius_a: 0.0,
      b: b[..],
      radius_b: 0.0,
      offset: 3.0,
      use_radii: false,
    );
    b3::DistanceInput overlapping = clear;
    overlapping.transform = a_placement_at({ 0.5, 0.0, 0.0 });

    return {
      .clear = @the_distance_of(b3::shape_distance(clear)),
      .overlapping = @the_distance_of(b3::shape_distance(overlapping)),
    };
}

fn void a_distance_query_asks_box3d_for_no_diagnostic_simplexes() @test {
    PointPairDistances points = the_distances_between_two_points_three_apart();
    BoxPairDiagnostics boxes = the_diagnostics_of_two_boxes();

    assert(points.surfaces.answered && boxes.clear.answered && boxes.overlapping.answered,
        "every pair answered, so the counters they carry are box3d's own");
    assert(points.surfaces.reported_no_simplexes && boxes.clear.reported_no_simplexes,
        "the wrapper passes no simplex array, so none is ever reported back");

    assert(1.7499 < boxes.clear.distance && boxes.clear.distance < 1.7501,
        "the two boxes stand a metre and three quarters apart");
    assert(math::abs(boxes.clear.point_a.x - 1.0f) < 0.00001
        && math::abs(boxes.clear.point_b.x - 2.75f) < 0.00001,
        "each witness point on its own box's face, which the two sizes place differently");

    assert(boxes.clear.iterations > 0,
        "a pair GJK has to walk towards counts the support points it added past the first");
    assert(points.surfaces.iterations == 0,
        "where a pair of single points is answered by that first simplex alone");
    assert(boxes.overlapping.iterations == 0,
        "and a core overlap reports none at all, box3d returning before the count is written");
}

struct ClosingDistances {
    DistanceReading clear;
    DistanceReading radii_crossed;
    DistanceReading cores_together;
}

// A metre of radius apiece against three offsets: clear of each other, overlapping by the radii
// while the cores stay apart, and the two cores in one place. Only the third is an overlap box3d
// resolves through GJK rather than through the radii.
fn ClosingDistances the_distances_as_two_metre_points_close() {
    b3::Vec3[1] a = { { 0.0, 0.0, 0.0 } };
    b3::Vec3[1] b = { { 0.0, 0.0, 0.0 } };

    b3::DistanceInput clear = a_point_pair(
      a: a[..],
      radius_a: 1.0,
      b: b[..],
      radius_b: 1.0,
      offset: 2.5,
      use_radii: true,
    );
    b3::DistanceInput crossed = clear;
    crossed.transform = a_placement_at({ 1.5, 0.0, 0.0 });
    b3::DistanceInput together = clear;
    together.transform = a_placement_at({ 0.0, 0.0, 0.0 });

    return {
      .clear = @the_distance_of(b3::shape_distance(clear)),
      .radii_crossed = @the_distance_of(b3::shape_distance(crossed)),
      .cores_together = @the_distance_of(b3::shape_distance(together)),
    };
}

fn void a_distance_the_radii_alone_drove_to_zero_still_names_a_direction() @test {
    ClosingDistances closing = the_distances_as_two_metre_points_close();

    assert(closing.clear.answered && closing.radii_crossed.answered, "both pairs answer");
    assert(0.4999 < closing.clear.distance && closing.clear.distance < 0.5001,
        "half a metre of surface separation while the two are still clear");
    assert(closing.radii_crossed.distance >= 0.0 && closing.radii_crossed.distance < 0.00001,
        "and zero once the radii cross, box3d clamping rather than reporting a negative overlap");

    assert(math::abs(closing.radii_crossed.normal.x - 1.0f) < 0.00001
        && math::abs(the_length_of(closing.radii_crossed.normal) - 1.0f) < 0.00001,
        "the direction survives that clamp and is still a unit vector");
    assert(closing.radii_crossed.point_b.x < closing.radii_crossed.point_a.x,
        "while the two perimeter points have crossed, B landing behind A along it");
}

fn void two_cores_in_one_place_report_no_direction_at_all() @test {
    ClosingDistances closing = the_distances_as_two_metre_points_close();

    assert(closing.cores_together.answered, "coincident cores are answered rather than refused");
    assert(closing.cores_together.distance >= 0.0 && closing.cores_together.distance < 0.00001,
        "at a distance of zero");
    assert(the_length_of(closing.cores_together.normal) < 0.00001,
        "under a normal of exactly zero, which a caller normalising it turns into a NaN");
    assert(the_length_of(closing.clear.normal) > 0.9999,
        "where a pair box3d could measure a direction between hands back a real one");
}

struct SweptPairFractions {
    CastReading no_radius;
    CastReading one_slop_of_radius;
    CastReading a_metre_of_radius;
}

// The cores start six metres apart and B travels twelve, so a pair carrying a total radius R
// touches at (6 - R) / 12. The three radii bracket box3d's own switch: it closes the cores to the
// greater of one linear slop and R less one slop, which puts the answer either side of that touch.
fn SweptPairFractions the_fractions_of_three_swept_pairs() {
    b3::Vec3[1] a = { { 0.0, 0.0, 0.0 } };
    b3::Vec3[1] b = { { 0.0, 0.0, 0.0 } };
    float half_a_slop = 0.5f * a_linear_slop();

    b3::ShapeCastPairInput bare = a_swept_point_pair(
      a: a[..],
      radius_a: 0.0,
      b: b[..],
      radius_b: 0.0,
      offset: 6.0,
      travel: -12.0,
      max_fraction: 1.0,
      can_encroach: false,
    );
    b3::ShapeCastPairInput slop_wide = bare;
    slop_wide.proxy_a.radius = half_a_slop;
    slop_wide.proxy_b.radius = half_a_slop;
    b3::ShapeCastPairInput metre_wide = bare;
    metre_wide.proxy_a.radius = 0.5;
    metre_wide.proxy_b.radius = 0.5;

    return {
      .no_radius = @the_reading_of(b3::shape_cast(bare)),
      .one_slop_of_radius = @the_reading_of(b3::shape_cast(slop_wide)),
      .a_metre_of_radius = @the_reading_of(b3::shape_cast(metre_wide)),
    };
}

fn void a_swept_pair_stops_past_the_touch_or_short_of_it_by_the_radius_it_carries() @test {
    SweptPairFractions swept = the_fractions_of_three_swept_pairs();
    float slop = a_linear_slop() / 12.0f;

    assert(swept.no_radius.hit && swept.one_slop_of_radius.hit && swept.a_metre_of_radius.hit,
        "all three pairs meet within the sweep");

    float bare_touch = 6.0f / 12.0f;
    assert(swept.no_radius.fraction < bare_touch - 0.5f * slop
        && swept.no_radius.fraction > bare_touch - 2.0f * slop,
        "a pair carrying no radius between them stops one linear slop short of touching");

    float metre_touch = (6.0f - 1.0f) / 12.0f;
    assert(swept.a_metre_of_radius.fraction > metre_touch + 0.5f * slop
        && swept.a_metre_of_radius.fraction < metre_touch + 2.0f * slop,
        "and one carrying a metre of it stops a slop past, so the direction turns on the radius");

    float slop_touch = (6.0f - a_linear_slop()) / 12.0f;
    assert(math::abs(swept.one_slop_of_radius.fraction - slop_touch) < 0.5f * slop,
        "with a total radius of exactly one slop landing on the touch, where the two rules meet");
}

struct SweptPairOutcomes {
    CastReading meeting;
    CastReading uneven;
    CastReading bounded_short;
    CastReading bounded_past;
    CastReading receding;
    CastReading passing_by;
}

// Half a metre of radius apiece, three metres apart, travelling six: the surfaces meet at 2 / 6.
// The uneven pair carries that same total on a different split and the two bounds straddle the
// fraction, while the last two never reach each other — one travelling away, one passing clear.
fn SweptPairOutcomes the_outcomes_of_a_pair_swept_six_metres() {
    b3::Vec3[1] a = { { 0.0, 0.0, 0.0 } };
    b3::Vec3[1] b = { { 0.0, 0.0, 0.0 } };

    b3::ShapeCastPairInput meeting = a_swept_point_pair(
      a: a[..],
      radius_a: 0.5,
      b: b[..],
      radius_b: 0.5,
      offset: 3.0,
      travel: -6.0,
      max_fraction: 1.0,
      can_encroach: false,
    );
    b3::ShapeCastPairInput uneven = meeting;
    uneven.proxy_a.radius = 0.25;
    uneven.proxy_b.radius = 0.75;
    b3::ShapeCastPairInput bounded_short = meeting;
    bounded_short.max_fraction = 0.3;
    b3::ShapeCastPairInput bounded_past = meeting;
    bounded_past.max_fraction = 0.4;
    b3::ShapeCastPairInput receding = meeting;
    receding.translation_b = { 6.0, 0.0, 0.0 };
    b3::ShapeCastPairInput passing_by = meeting;
    passing_by.transform = a_placement_at({ 3.0, 3.0, 0.0 });

    return {
      .meeting = @the_reading_of(b3::shape_cast(meeting)),
      .uneven = @the_reading_of(b3::shape_cast(uneven)),
      .bounded_short = @the_reading_of(b3::shape_cast(bounded_short)),
      .bounded_past = @the_reading_of(b3::shape_cast(bounded_past)),
      .receding = @the_reading_of(b3::shape_cast(receding)),
      .passing_by = @the_reading_of(b3::shape_cast(passing_by)),
    };
}

fn void a_swept_pair_reports_the_fraction_at_which_the_two_first_touch() @test {
    SweptPairOutcomes swept = the_outcomes_of_a_pair_swept_six_metres();
    float touch = 2.0f / 6.0f;
    float slop = a_linear_slop() / 6.0f;

    assert(swept.meeting.hit, "the two meet within the sweep");
    assert(swept.meeting.fraction > touch && swept.meeting.fraction < touch + 2.0f * slop,
        "a third of the way along, a linear slop past where the surfaces meet");
    assert(math::abs(swept.meeting.point.x - 0.5f) < 0.00001,
        "the point sitting on A's own surface rather than between the two");
    assert(math::abs(swept.meeting.normal.x - 1.0f) < 0.00001
        && math::abs(the_length_of(swept.meeting.normal) - 1.0f) < 0.00001,
        "under a unit normal running from A towards B");
    assert(swept.meeting.iterations > 0, "and box3d counts the passes it took to close on it");

    assert(swept.uneven.hit && math::abs(swept.uneven.fraction - swept.meeting.fraction) < 0.00001,
        "the same total radius split unevenly meets at the same fraction");
    assert(math::abs(swept.uneven.point.x - 0.25f) < 0.00001,
        "with the point still on A's surface, which that split has moved");
}

fn void a_swept_pair_that_never_reaches_the_other_reports_no_hit() @test {
    SweptPairOutcomes swept = the_outcomes_of_a_pair_swept_six_metres();

    assert(!swept.receding.hit, "a pair travelling apart never touches");
    assert(!swept.passing_by.hit, "nor does one passing three metres clear");
    assert(!swept.bounded_short.hit,
        "and a touch beyond max_fraction is a miss, not a hit clamped to that bound");
    assert(swept.bounded_past.hit && math::abs(swept.bounded_past.fraction - swept.meeting.fraction) < 0.00001,
        "where a bound past the touch leaves the same fraction the unbounded sweep gives");
}

struct OverlappedSweep {
    CastReading plain;
    CastReading encroaching;
}

// Half a metre of radius apiece with the cores only half a metre apart, so the pair begins well
// inside the separation box3d closes on. can_encroach is the only thing that differs.
fn OverlappedSweep the_sweep_of_a_pair_that_begins_overlapped() {
    b3::Vec3[1] a = { { 0.0, 0.0, 0.0 } };
    b3::Vec3[1] b = { { 0.0, 0.0, 0.0 } };

    b3::ShapeCastPairInput plain = a_swept_point_pair(
      a: a[..],
      radius_a: 0.5,
      b: b[..],
      radius_b: 0.5,
      offset: 0.5,
      travel: -6.0,
      max_fraction: 1.0,
      can_encroach: false,
    );
    b3::ShapeCastPairInput encroaching = plain;
    encroaching.can_encroach = true;

    return {
      .plain = @the_reading_of(b3::shape_cast(plain)),
      .encroaching = @the_reading_of(b3::shape_cast(encroaching)),
    };
}

fn void a_pair_that_begins_overlapped_reports_a_hit_that_has_gone_nowhere() @test {
    OverlappedSweep overlapped = the_sweep_of_a_pair_that_begins_overlapped();

    assert(overlapped.plain.hit, "an overlap at the outset is a hit rather than a miss");
    assert(math::abs(overlapped.plain.fraction) < 0.0000001, "at fraction zero, the sweep having not moved");
    assert(the_length_of(overlapped.plain.normal) < 0.00001,
        "under a normal of exactly zero, which a caller normalising it turns into a NaN");

    assert(overlapped.encroaching.hit, "can_encroach still reports the hit");
    float slop = a_linear_slop() / 6.0f;
    assert(overlapped.encroaching.fraction > 0.5f * slop && overlapped.encroaching.fraction < 2.0f * slop,
        "and lets the pair close a further linear slop instead of answering where it started");
    assert(math::abs(the_length_of(overlapped.encroaching.normal) - 1.0f) < 0.00001,
        "which is what puts a real direction on the answer");
}

struct ProxyPairRefusals {
    fault distance_without_a_first_proxy;
    fault distance_without_a_second_proxy;
    fault sweep_without_a_first_proxy;
    fault sweep_without_a_second_proxy;
    bool distance_still_answers;
    bool sweep_still_answers;
}

// An empty proxy rather than a null one, because box3d reads a count of none as though it held its
// first point and so answers plausibly about geometry the caller never named. Each refusal leaves
// the other proxy whole, so a check dropped from one of the two cannot be covered by the other.
fn ProxyPairRefusals the_proxy_pairs_the_two_queries_refuse() {
    b3::Vec3[1] a = { { 0.0, 0.0, 0.0 } };
    b3::Vec3[1] b = { { 0.0, 0.0, 0.0 } };

    b3::DistanceInput measured = a_point_pair(
      a: a[..],
      radius_a: 0.5,
      b: b[..],
      radius_b: 0.5,
      offset: 3.0,
      use_radii: true,
    );
    b3::DistanceInput no_first = measured;
    no_first.proxy_a.count = 0;
    b3::DistanceInput no_second = measured;
    no_second.proxy_b.count = 0;

    b3::ShapeCastPairInput swept = a_swept_point_pair(
      a: a[..],
      radius_a: 0.5,
      b: b[..],
      radius_b: 0.5,
      offset: 3.0,
      travel: -6.0,
      max_fraction: 1.0,
      can_encroach: false,
    );
    b3::ShapeCastPairInput swept_no_first = swept;
    swept_no_first.proxy_a.count = 0;
    b3::ShapeCastPairInput swept_no_second = swept;
    swept_no_second.proxy_b.count = 0;

    ProxyPairRefusals refusals;
    if (catch err = b3::shape_distance(no_first)) refusals.distance_without_a_first_proxy = err;
    if (catch err = b3::shape_distance(no_second)) refusals.distance_without_a_second_proxy = err;
    if (catch err = b3::shape_cast(swept_no_first)) refusals.sweep_without_a_first_proxy = err;
    if (catch err = b3::shape_cast(swept_no_second)) refusals.sweep_without_a_second_proxy = err;

    if (try out = b3::shape_distance(measured)) refusals.distance_still_answers = out.distance > 1.0;
    if (try out = b3::shape_cast(swept)) refusals.sweep_still_answers = out.fraction > 0.0;

    return refusals;
}

fn void both_proxy_pair_queries_refuse_a_proxy_box3d_would_answer_wrongly() @test {
    ProxyPairRefusals refused = the_proxy_pairs_the_two_queries_refuse();

    assert(refused.distance_without_a_first_proxy == b3::INVALID_DEFINITION,
        "a distance query will not take a first proxy of no points");
    assert(refused.distance_without_a_second_proxy == b3::INVALID_DEFINITION, "nor a second");
    assert(refused.sweep_without_a_first_proxy == b3::INVALID_DEFINITION,
        "and neither will a swept pair take a first proxy of no points");
    assert(refused.sweep_without_a_second_proxy == b3::INVALID_DEFINITION, "nor a second");

    assert(refused.distance_still_answers, "where a pair of ordinary proxies is measured");
    assert(refused.sweep_still_answers, "and swept");
}

fn void the_proxy_pair_queries_allocate_nothing() @test {
    int before = b3::byte_count();

    PointPairDistances points = the_distances_between_two_points_three_apart();
    ClosingDistances closing = the_distances_as_two_metre_points_close();
    SweptPairFractions fractions = the_fractions_of_three_swept_pairs();
    SweptPairOutcomes outcomes = the_outcomes_of_a_pair_swept_six_metres();
    OverlappedSweep overlapped = the_sweep_of_a_pair_that_begins_overlapped();
    BoxPairDiagnostics boxes = the_diagnostics_of_two_boxes();
    the_proxy_pairs_the_two_queries_refuse();

    assert(points.cores.answered && closing.clear.answered && boxes.clear.answered,
        "the distance queries all ran");
    assert(fractions.no_radius.hit && outcomes.meeting.hit && overlapped.plain.hit, "as did the sweeps");
    assert(b3::byte_count() == before,
        "and neither query allocates: both answer from the caller's own proxies and stack");
}
