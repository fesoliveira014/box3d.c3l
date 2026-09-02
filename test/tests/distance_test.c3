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

// A quarter turn, which every sweep fixture below runs through. Written out rather than quoted so
// the closed forms the assertions compare against are the same expression box3d is handed.
const float A_QUARTER_TURN = (float)math::PI * 0.5f;
const float HALF_ROOT_TWO = 0.70710678f;

fn b3::Quat a_turn_about_z(float radians) {
    return { .i = 0.0, .j = 0.0, .k = math::sin(0.5f * radians), .l = math::cos(0.5f * radians) };
}

fn float the_turn_of(b3::Quat q) {
    return 2.0f * math::atan2(math::sqrt(q.i * q.i + q.j * q.j + q.k * q.k), q.l);
}

fn float the_magnitude_of(b3::Quat q) {
    return math::sqrt(q.i * q.i + q.j * q.j + q.k * q.k + q.l * q.l);
}

// The normalised lerp's own closed form for a turn about z: the two quaternions are summed in the
// ratio the fraction names and the sum renormalised, which leaves the angle of the raw sum.
fn float the_normalised_lerp_turn_at(float fraction, float total_turn) {
    float k = fraction * math::sin(0.5f * total_turn);
    float l = (1.0f - fraction) + fraction * math::cos(0.5f * total_turn);
    return 2.0f * math::atan2(k, l);
}

// What one sweep evaluation answered. answered is whether the wrapper returned a value rather than
// a fault, and magnitude is the quaternion's own length, which an identity rotation still has to
// carry and a reading left at its container's zero cannot.
struct SweepReading {
    bool answered;
    b3::Vec3 position;
    float turn;
    float magnitude;
}

// An expression macro for the reason @the_distance_of is one: C3 forbids an optional parameter, so
// the call has to arrive unevaluated to keep a refusal from propagating out of the fixture.
macro SweepReading @the_pose_at(#call) {
    if (try xf = #call) {
        return {
          .answered = true,
          .position = xf.p,
          .turn = the_turn_of(xf.q),
          .magnitude = the_magnitude_of(xf.q),
        };
    }
    return {};
}

struct SweptPoses {
    SweepReading at_the_start;
    SweepReading at_a_quarter;
    SweepReading at_the_midpoint;
    SweepReading at_the_end;
}

// A quarter turn about z while the centre of mass runs ten metres along x, with the shape's own
// origin a metre out from that centre along its local x. The two therefore never coincide, so a
// reading that mistook the centre for the origin lands a metre from every expectation below.
fn b3::Sweep a_turning_sweep() {
    return {
      .local_center = { 1.0, 0.0, 0.0 },
      .c1 = { 0.0, 0.0, 0.0 },
      .c2 = { 10.0, 0.0, 0.0 },
      .q1 = a_turn_about_z(0.0),
      .q2 = a_turn_about_z(A_QUARTER_TURN),
    };
}

fn SweptPoses the_poses_along_a_turning_sweep() {
    b3::Sweep sweep = a_turning_sweep();

    return {
      .at_the_start = @the_pose_at(sweep.transform_at(0.0)),
      .at_a_quarter = @the_pose_at(sweep.transform_at(0.25)),
      .at_the_midpoint = @the_pose_at(sweep.transform_at(0.5)),
      .at_the_end = @the_pose_at(sweep.transform_at(1.0)),
    };
}

fn void a_sweep_places_the_shape_origin_at_both_ends_of_its_interval() @test {
    SweptPoses poses = the_poses_along_a_turning_sweep();

    assert(poses.at_the_start.answered && poses.at_the_end.answered,
        "both ends of the interval are inside it and answer");

    assert(the_length_of(poses.at_the_start.position - (b3::Vec3){ -1.0, 0.0, 0.0 }) < 0.0001,
        "at the start the origin sits a metre behind the centre, which is where local_center puts it");
    assert(math::abs(poses.at_the_start.turn) < 0.0001
        && math::abs(poses.at_the_start.magnitude - 1.0f) < 0.0001,
        "under q1 itself, a unit quaternion turned through nothing");

    assert(the_length_of(poses.at_the_end.position - (b3::Vec3){ 10.0, -1.0, 0.0 }) < 0.0001,
        "at the end the centre has run ten metres and the turn has swung that metre onto y");
    assert(math::abs(poses.at_the_end.turn - A_QUARTER_TURN) < 0.0001
        && math::abs(poses.at_the_end.magnitude - 1.0f) < 0.0001,
        "under q2, the whole quarter turn");
}

fn void a_sweep_turns_on_a_normalised_lerp_that_only_its_midpoint_hides() @test {
    SweptPoses poses = the_poses_along_a_turning_sweep();
    float lerped = the_normalised_lerp_turn_at(0.25, A_QUARTER_TURN);
    float slerped = 0.25f * A_QUARTER_TURN;

    assert(poses.at_a_quarter.answered && poses.at_the_midpoint.answered,
        "both fractions are inside the interval and answer");

    assert(math::abs(poses.at_a_quarter.turn - lerped) < 0.0001
        && math::abs(poses.at_a_quarter.magnitude - 1.0f) < 0.0001,
        "a quarter of the way through, the rotation is the normalised lerp of the two quaternions");
    assert(poses.at_a_quarter.turn < slerped - 0.01 && poses.at_a_quarter.turn > slerped - 0.03,
        "which lags the slerp of the same arc, the lerp turning slowest at the two ends");

    assert(math::abs(poses.at_the_midpoint.turn - 0.5f * A_QUARTER_TURN) < 0.0001,
        "while at the midpoint the two agree exactly, so a fixture that sampled only there could not tell them apart");
    assert(the_length_of(poses.at_the_midpoint.position
        - (b3::Vec3){ 5.0f - HALF_ROOT_TWO, -HALF_ROOT_TWO, 0.0 }) < 0.0001,
        "with the origin swung half the turn out from a centre halfway along");
}

struct SweepRefusals {
    fault before_the_start;
    fault past_the_end;
    fault at_no_number;
    bool inside_still_answers;
}

// box3d asserts the fraction and this build compiles that out, so each of these is answered rather
// than refused underneath: a negative one runs the sweep backwards, one past the end runs it on
// past c2, and one that is not a number reaches the interpolation whole.
fn SweepRefusals the_fractions_a_sweep_refuses() {
    b3::Sweep sweep = a_turning_sweep();
    float not_a_number = 0.0f / 0.0f;

    SweepRefusals refusals;
    if (catch err = sweep.transform_at(-1.0)) refusals.before_the_start = err;
    if (catch err = sweep.transform_at(2.0)) refusals.past_the_end = err;
    if (catch err = sweep.transform_at(not_a_number)) refusals.at_no_number = err;

    if (try xf = sweep.transform_at(0.5)) refusals.inside_still_answers = xf.p.x > 4.0;
    return refusals;
}

fn void a_sweep_refuses_a_fraction_outside_its_own_interval() @test {
    SweepRefusals refused = the_fractions_a_sweep_refuses();

    assert(refused.before_the_start == b3::INVALID_DEFINITION,
        "a negative fraction is refused rather than run backwards out of the interval");
    assert(refused.past_the_end == b3::INVALID_DEFINITION,
        "as is one past the end, which box3d would carry on past c2");
    assert(refused.at_no_number == b3::INVALID_DEFINITION,
        "and so is one that is not a number, which a pair of bare comparisons would let through");

    assert(refused.inside_still_answers, "while a fraction inside the interval is answered");
}

// What one time-of-impact query answered. answered is whether the wrapper returned a value rather
// than a fault; the state is box3d's own and no reachable one of them is the enum's zero, so a
// reading left at its container's zero fails every assertion that names a state.
struct ImpactReading {
    bool answered;
    b3::ToiState state;
    float fraction;
    b3::Vec3 point;
    b3::Vec3 normal;
    float distance;
}

macro ImpactReading @the_impact_of(#call) {
    if (try out = #call) {
        return {
          .answered = true,
          .state = out.state,
          .fraction = out.fraction,
          .point = out.point,
          .normal = out.normal,
          .distance = out.distance,
        };
    }
    return {};
}

// Both proxies keep their points at their own origin and the sweeps carry them, so a pair whose
// cores start gap apart and close by travel has its cores meet at gap / travel. The points are
// borrowed for the length of the call, so each fixture holds its own arrays across it.
fn b3::ToiInput a_swept_proxy_pair(
  b3::Vec3[] a,
  float radius_a,
  b3::Vec3[] b,
  float radius_b,
  float gap,
  float travel,
  float max_fraction,
) {
    b3::Quat unturned = a_turn_about_z(0.0);
    return {
      .proxy_a = { .points = a.ptr, .count = (int)a.len, .radius = radius_a },
      .proxy_b = { .points = b.ptr, .count = (int)b.len, .radius = radius_b },
      .sweep_a = {
        .local_center = { 0.0, 0.0, 0.0 },
        .c1 = { 0.0, 0.0, 0.0 },
        .c2 = { 0.0, 0.0, 0.0 },
        .q1 = unturned,
        .q2 = unturned,
      },
      .sweep_b = {
        .local_center = { 0.0, 0.0, 0.0 },
        .c1 = { gap, 0.0, 0.0 },
        .c2 = { gap - travel, 0.0, 0.0 },
        .q1 = unturned,
        .q2 = unturned,
      },
      .max_fraction = max_fraction,
    };
}

struct SweptImpacts {
    ImpactReading meeting;
    ImpactReading radii_crossed;
    ImpactReading cores_together;
    ImpactReading receding;
    ImpactReading bounded_short;
}

// Half a metre of radius apiece with the cores six metres apart and closing six, so the surfaces
// meet at five sixths. The next two begin already inside that — one by its radii alone, one with
// the two cores in the same place — and the last two never meet, one receding and one bounded.
fn SweptImpacts the_impacts_of_a_pair_swept_six_metres() {
    b3::Vec3[1] a = { { 0.0, 0.0, 0.0 } };
    b3::Vec3[1] b = { { 0.0, 0.0, 0.0 } };

    b3::ToiInput meeting = a_swept_proxy_pair(
      a: a[..],
      radius_a: 0.5,
      b: b[..],
      radius_b: 0.5,
      gap: 6.0,
      travel: 6.0,
      max_fraction: 1.0,
    );
    b3::ToiInput radii_crossed = meeting;
    radii_crossed.sweep_b.c1 = { 0.5, 0.0, 0.0 };
    radii_crossed.sweep_b.c2 = { -5.5, 0.0, 0.0 };
    b3::ToiInput cores_together = meeting;
    cores_together.sweep_b.c1 = { 0.0, 0.0, 0.0 };
    cores_together.sweep_b.c2 = { -6.0, 0.0, 0.0 };
    b3::ToiInput receding = meeting;
    receding.sweep_b.c2 = { 12.0, 0.0, 0.0 };
    b3::ToiInput bounded_short = meeting;
    bounded_short.max_fraction = 0.5;

    return {
      .meeting = @the_impact_of(b3::time_of_impact(meeting)),
      .radii_crossed = @the_impact_of(b3::time_of_impact(radii_crossed)),
      .cores_together = @the_impact_of(b3::time_of_impact(cores_together)),
      .receding = @the_impact_of(b3::time_of_impact(receding)),
      .bounded_short = @the_impact_of(b3::time_of_impact(bounded_short)),
    };
}

fn void a_swept_pair_reports_the_time_at_which_it_first_touches() @test {
    SweptImpacts swept = the_impacts_of_a_pair_swept_six_metres();
    float touch = 5.0f / 6.0f;
    float slop_of_travel = a_linear_slop() / 6.0f;

    assert(swept.meeting.answered && swept.meeting.state == b3::ToiState.HIT,
        "a pair that reaches the other inside the interval reports a hit");
    assert(swept.meeting.fraction > touch + 0.5f * slop_of_travel
        && swept.meeting.fraction < touch + 2.0f * slop_of_travel,
        "five sixths of the way along, a linear slop of travel past where the surfaces meet");

    assert(math::abs(swept.meeting.distance - (1.0f - a_linear_slop())) < 0.25f * a_linear_slop(),
        "the cores having closed to the two radii less a slop, which is where box3d stops them");
    assert(math::abs(swept.meeting.point.x - (0.5f - 0.5f * a_linear_slop())) < 0.25f * a_linear_slop(),
        "the point averaging two surface points that have crossed by that slop");
    assert(math::abs(swept.meeting.normal.x - 1.0f) < 0.0001
        && math::abs(the_length_of(swept.meeting.normal) - 1.0f) < 0.0001,
        "under a unit normal running from A towards B");
}

fn void a_pair_that_begins_inside_the_other_answers_by_what_of_it_overlaps() @test {
    SweptImpacts swept = the_impacts_of_a_pair_swept_six_metres();

    assert(swept.radii_crossed.state == b3::ToiState.HIT
        && math::abs(swept.radii_crossed.fraction) < 0.0000001,
        "a pair overlapping by its radii alone is a hit that has gone nowhere");
    assert(swept.radii_crossed.state == b3::ToiState.HIT
        && math::abs(swept.radii_crossed.normal.x - 1.0f) < 0.0001
        && math::abs(swept.radii_crossed.point.x - 0.25f) < 0.0001,
        "reported over a measured direction and a point between the two crossed surfaces");
    assert(0.4999 < swept.radii_crossed.distance && swept.radii_crossed.distance < 0.5001,
        "its cores still half a metre apart, which is what box3d could measure between");

    assert(swept.cores_together.state == b3::ToiState.OVERLAPPED
        && math::abs(swept.cores_together.fraction) < 0.0000001,
        "where two cores in one place are overlapped rather than hit, also at fraction zero");
    assert(swept.cores_together.state == b3::ToiState.OVERLAPPED
        && the_length_of(swept.cores_together.normal) < 0.0001
        && the_length_of(swept.cores_together.point) < 0.0001,
        "box3d giving up on the pair and reporting neither a direction nor a place");
}

fn void a_swept_pair_that_never_reaches_the_other_is_separated_rather_than_refused() @test {
    SweptImpacts swept = the_impacts_of_a_pair_swept_six_metres();

    assert(swept.receding.state == b3::ToiState.SEPARATED
        && math::abs(swept.receding.fraction - 1.0f) < 0.0001,
        "a pair travelling apart is separated at the end of the interval rather than at a touch");
    assert(5.9999 < swept.receding.distance && swept.receding.distance < 6.0001,
        "over the witness pair the last distance query left, six metres apart where the two started");
    assert(math::abs(swept.receding.point.x - 3.0f) < 0.0001,
        "whose midpoint is halfway between them, not a place either surface ever reached");

    assert(swept.bounded_short.state == b3::ToiState.SEPARATED
        && math::abs(swept.bounded_short.fraction - 0.5f) < 0.0001,
        "and a touch past max_fraction is separated at that bound, not a hit clamped back to it");
}

struct SweptPairRefusals {
    fault without_a_first_proxy;
    fault without_a_second_proxy;
    fault below_the_interval;
    fault past_the_interval;
    fault at_no_number;
    bool still_answers;
}

// A negative max_fraction is answered underneath at a negative fraction, one past the end runs
// both sweeps past c2, and one that is not a number burns every iteration box3d has. Each proxy
// refusal leaves the other whole, so a check dropped from one is not covered by the other.
fn SweptPairRefusals the_swept_pairs_the_time_of_impact_refuses() {
    b3::Vec3[1] a = { { 0.0, 0.0, 0.0 } };
    b3::Vec3[1] b = { { 0.0, 0.0, 0.0 } };
    float not_a_number = 0.0f / 0.0f;

    b3::ToiInput swept = a_swept_proxy_pair(
      a: a[..],
      radius_a: 0.5,
      b: b[..],
      radius_b: 0.5,
      gap: 6.0,
      travel: 6.0,
      max_fraction: 1.0,
    );
    b3::ToiInput no_first = swept;
    no_first.proxy_a.count = 0;
    b3::ToiInput no_second = swept;
    no_second.proxy_b.count = 0;
    b3::ToiInput below = swept;
    below.max_fraction = -1.0;
    b3::ToiInput past = swept;
    past.max_fraction = 3.0;
    b3::ToiInput no_number = swept;
    no_number.max_fraction = not_a_number;

    SweptPairRefusals refusals;
    if (catch err = b3::time_of_impact(no_first)) refusals.without_a_first_proxy = err;
    if (catch err = b3::time_of_impact(no_second)) refusals.without_a_second_proxy = err;
    if (catch err = b3::time_of_impact(below)) refusals.below_the_interval = err;
    if (catch err = b3::time_of_impact(past)) refusals.past_the_interval = err;
    if (catch err = b3::time_of_impact(no_number)) refusals.at_no_number = err;

    if (try out = b3::time_of_impact(swept)) refusals.still_answers = out.fraction > 0.8;
    return refusals;
}

fn void the_time_of_impact_refuses_what_box3d_would_answer_out_of_its_own_interval() @test {
    SweptPairRefusals refused = the_swept_pairs_the_time_of_impact_refuses();

    assert(refused.without_a_first_proxy == b3::INVALID_DEFINITION,
        "a swept pair will not take a first proxy of no points");
    assert(refused.without_a_second_proxy == b3::INVALID_DEFINITION, "nor a second");

    assert(refused.below_the_interval == b3::INVALID_DEFINITION,
        "a negative max_fraction is refused rather than answered at a negative fraction");
    assert(refused.past_the_interval == b3::INVALID_DEFINITION,
        "as is one past the end, which would run both sweeps beyond c2");
    assert(refused.at_no_number == b3::INVALID_DEFINITION,
        "and so is one that is not a number, which a pair of bare comparisons would let through");

    assert(refused.still_answers, "while an ordinary pair over the whole interval is answered");
}

fn void the_sweep_and_the_time_of_impact_allocate_nothing() @test {
    int before = b3::byte_count();

    SweptPoses poses = the_poses_along_a_turning_sweep();
    SweptImpacts impacts = the_impacts_of_a_pair_swept_six_metres();
    the_fractions_a_sweep_refuses();
    the_swept_pairs_the_time_of_impact_refuses();

    assert(poses.at_the_start.answered && poses.at_the_end.answered, "every pose was evaluated");
    assert(impacts.meeting.state == b3::ToiState.HIT
        && impacts.cores_together.state == b3::ToiState.OVERLAPPED, "and every sweep was run");
    assert(b3::byte_count() == before,
        "and neither answers from anything but the caller's own proxies and stack");
}
