module b3_test;

import b3;
import std::math;

// What one collide call answered, flattened so a fixture can hand back a contact and a miss
// together. slice_len comes from contact_points() rather than from point_count, so the two
// readings disagreeing is what a wrong prefix would look like.
struct ManifoldReading {
    int point_count;
    usz slice_len;
    b3::Vec3 normal;
    b3::LocalManifoldPoint[b3::MAX_MANIFOLD_POINTS] points;
}

fn ManifoldReading the_reading_of(b3::CollisionManifold manifold) {
    ManifoldReading reading = {
      .point_count = manifold.point_count,
      .slice_len = manifold.contact_points().len,
      .normal = manifold.normal,
    };
    foreach (index, point : manifold.contact_points()) {
        reading.points[index] = point;
    }
    return reading;
}

fn b3::Sphere a_half_metre_sphere() {
    return { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
}

fn b3::Capsule a_metre_capsule_along_x() {
    return { .center1 = { -0.5, 0.0, 0.0 }, .center2 = { 0.5, 0.0, 0.0 }, .radius = 0.25 };
}

fn b3::Transform a_placement_at(b3::Vec3 offset) {
    return { .p = offset, .q = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 } };
}

fn float the_length_of(b3::Vec3 v) {
    return math::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

struct SpherePairReadings {
    ManifoldReading overlapping;
    ManifoldReading apart;
    ManifoldReading just_clear;
}

// The contact is read first and the two misses after it, which is the order that would surface a
// manifold carrying an earlier call's answer. "Just clear" leaves a hundredth of a metre, half of
// box3d's speculative distance, so a pair that had a speculative window would still report there.
fn SpherePairReadings the_readings_of_two_half_metre_spheres() {
    b3::Sphere a = a_half_metre_sphere();
    b3::Sphere b = a_half_metre_sphere();

    return {
      .overlapping = the_reading_of(b3::collide_spheres(&a, b, a_placement_at({ 0.75, 0.0, 0.0 }))),
      .apart = the_reading_of(b3::collide_spheres(&a, b, a_placement_at({ 3.0, 0.0, 0.0 }))),
      .just_clear = the_reading_of(b3::collide_spheres(&a, b, a_placement_at({ 1.01, 0.0, 0.0 }))),
    };
}

fn void two_overlapping_spheres_report_the_point_between_their_surfaces() @test {
    SpherePairReadings spheres = the_readings_of_two_half_metre_spheres();
    b3::LocalManifoldPoint contact = spheres.overlapping.points[0];

    assert(spheres.overlapping.point_count == 1, "two spheres meet at one point");
    assert(spheres.overlapping.slice_len == 1, "and contact_points() hands back exactly that point");

    float expected_separation = 0.75f - (0.5f + 0.5f);
    assert(-0.2501 < contact.separation && contact.separation < -0.2499,
        "the separation is the gap between the centres less the two radii, negative for an overlap");
    assert(math::abs(contact.separation - expected_separation) < 0.00001);

    float expected_x = 0.5f * (0.5f + (0.75f - 0.5f));
    assert(math::abs(contact.point.x - expected_x) < 0.00001,
        "and the point sits half way between the two surfaces along the normal");
    assert(math::abs(contact.point.y) < 0.00001 && math::abs(contact.point.z) < 0.00001);

    assert(0.9999 < spheres.overlapping.normal.x && spheres.overlapping.normal.x < 1.0001,
        "the normal runs from this sphere towards the other");
    assert(math::abs(spheres.overlapping.normal.y) < 0.00001 && math::abs(spheres.overlapping.normal.z) < 0.00001);
    assert(math::abs(the_length_of(spheres.overlapping.normal) - 1.0f) < 0.00001,
        "and it is a unit vector where the point is a position");
}

fn void a_sphere_pair_reports_nothing_once_the_two_are_clear() @test {
    SpherePairReadings spheres = the_readings_of_two_half_metre_spheres();

    assert(spheres.apart.point_count == 0, "two spheres three metres apart meet nowhere");
    assert(spheres.apart.slice_len == 0, "and contact_points() is empty rather than short");
    assert(the_length_of(spheres.apart.normal) < 0.00001,
        "a miss read after a contact carries no normal, where box3d would have left the last one standing");

    assert(spheres.just_clear.point_count == 0,
        "and a hundredth of a metre of daylight is a miss too: this pair has no speculative window");
    assert(spheres.just_clear.slice_len == 0);
}

struct CapsuleAndSphereReadings {
    ManifoldReading overlapping;
    ManifoldReading just_clear;
    ManifoldReading centres_coincident;
}

// The sphere sits 0.6 above a capsule axis that runs through the origin, against a summed radius
// of 0.75. The third reading collapses the capsule onto a single centre, which is inside the
// length box3d refuses to collide two capsules at.
fn CapsuleAndSphereReadings the_readings_of_a_capsule_under_a_sphere() {
    b3::Capsule capsule = a_metre_capsule_along_x();
    b3::Capsule collapsed = { .center1 = { 0.0, 0.0, 0.0 }, .center2 = { 0.0, 0.0, 0.0 }, .radius = 0.25 };
    b3::Sphere sphere = a_half_metre_sphere();

    return {
      .overlapping = the_reading_of(b3::collide_capsule_and_sphere(&capsule, sphere, a_placement_at({ 0.0, 0.6, 0.0 }))),
      .just_clear = the_reading_of(b3::collide_capsule_and_sphere(&capsule, sphere, a_placement_at({ 0.0, 0.76, 0.0 }))),
      .centres_coincident = the_reading_of(b3::collide_capsule_and_sphere(&collapsed, sphere, a_placement_at({ 0.0, 0.6, 0.0 }))),
    };
}

fn void a_capsule_and_a_sphere_report_the_point_between_them() @test {
    CapsuleAndSphereReadings pair = the_readings_of_a_capsule_under_a_sphere();
    b3::LocalManifoldPoint contact = pair.overlapping.points[0];

    assert(pair.overlapping.point_count == 1, "a capsule and a sphere meet at one point");
    assert(pair.overlapping.slice_len == 1);

    float expected_separation = 0.6f - (0.25f + 0.5f);
    assert(-0.1501 < contact.separation && contact.separation < -0.1499,
        "the separation is the distance to the capsule axis less the two radii");
    assert(math::abs(contact.separation - expected_separation) < 0.00001);

    float expected_y = 0.5f * ((0.6f - 0.5f) + 0.25f);
    assert(math::abs(contact.point.y - expected_y) < 0.00001,
        "and the point sits half way between the sphere's surface and the capsule's");
    assert(math::abs(contact.point.x) < 0.00001 && math::abs(contact.point.z) < 0.00001);

    assert(0.9999 < pair.overlapping.normal.y && pair.overlapping.normal.y < 1.0001,
        "the normal runs from the capsule towards the sphere");
    assert(math::abs(the_length_of(pair.overlapping.normal) - 1.0f) < 0.00001);

    assert(pair.just_clear.point_count == 0,
        "a hundredth of a metre of daylight is a miss: this pair has no speculative window either");
}

fn void a_capsule_with_coincident_centres_is_collided_as_the_sphere_it_names() @test {
    CapsuleAndSphereReadings pair = the_readings_of_a_capsule_under_a_sphere();

    assert(pair.centres_coincident.point_count == 1,
        "collapsing a capsule onto one centre leaves a sphere a sphere still meets");
    assert(math::abs(pair.centres_coincident.points[0].separation - pair.overlapping.points[0].separation) < 0.00001,
        "and it answers exactly where the metre-long capsule's own middle did");
    assert(math::abs(pair.centres_coincident.points[0].point.y - 0.175f) < 0.00001);
}

struct HullAndSphereReadings {
    ManifoldReading overlapping;
    ManifoldReading just_clear;
    ManifoldReading centre_inside;
    ManifoldReading apart;
}

// A half-metre box hull against a half-metre sphere raised above it. The box hull is returned by
// value and owns its own storage, so nothing here allocates or has to be destroyed.
fn HullAndSphereReadings the_readings_of_a_box_hull_under_a_sphere() {
    b3::BoxHull box = b3::make_box_hull(0.5, 0.5, 0.5);
    b3::Sphere sphere = a_half_metre_sphere();

    return {
      .overlapping = the_reading_of(b3::collide_hull_and_sphere(&box.base, sphere, a_placement_at({ 0.0, 0.9, 0.0 }))),
      .just_clear = the_reading_of(b3::collide_hull_and_sphere(&box.base, sphere, a_placement_at({ 0.0, 1.01, 0.0 }))),
      .centre_inside = the_reading_of(b3::collide_hull_and_sphere(&box.base, sphere, a_placement_at({ 0.0, 0.2, 0.0 }))),
      .apart = the_reading_of(b3::collide_hull_and_sphere(&box.base, sphere, a_placement_at({ 0.0, 3.0, 0.0 }))),
    };
}

fn void a_hull_and_a_sphere_report_the_point_between_them() @test {
    HullAndSphereReadings pair = the_readings_of_a_box_hull_under_a_sphere();
    b3::LocalManifoldPoint contact = pair.overlapping.points[0];

    assert(pair.overlapping.point_count == 1, "a hull and a sphere meet at one point");
    assert(pair.overlapping.slice_len == 1);

    float expected_separation = (0.9f - 0.5f) - 0.5f;
    assert(-0.1001 < contact.separation && contact.separation < -0.0999,
        "the separation is the distance from the centre to the face less the sphere's radius");
    assert(math::abs(contact.separation - expected_separation) < 0.00001);

    float expected_y = 0.5f * (0.5f + (0.9f - 0.5f));
    assert(math::abs(contact.point.y - expected_y) < 0.00001,
        "and the point sits half way between the face and the sphere's deepest point");
    assert(math::abs(contact.point.x) < 0.00001 && math::abs(contact.point.z) < 0.00001);

    assert(0.9999 < pair.overlapping.normal.y && pair.overlapping.normal.y < 1.0001,
        "the normal runs from the hull towards the sphere");
    assert(math::abs(the_length_of(pair.overlapping.normal) - 1.0f) < 0.00001);

    assert(pair.apart.point_count == 0, "three metres up is a miss");
    assert(the_length_of(pair.apart.normal) < 0.00001);
}

fn void a_hull_and_a_sphere_report_a_point_before_the_two_touch() @test {
    HullAndSphereReadings pair = the_readings_of_a_box_hull_under_a_sphere();
    b3::LocalManifoldPoint speculative = pair.just_clear.points[0];

    assert(pair.just_clear.point_count == 1,
        "this pair carries a speculative window, so a hundredth of a metre of daylight still reports");
    assert(0.0099 < speculative.separation && speculative.separation < 0.0101,
        "and the separation it reports is positive, being real daylight rather than an overlap");

    float expected_y = 0.5f * (0.5f + (1.01f - 0.5f));
    assert(math::abs(speculative.point.y - expected_y) < 0.00001);
}

fn void a_sphere_whose_centre_is_inside_a_hull_is_answered_from_the_nearest_face() @test {
    HullAndSphereReadings pair = the_readings_of_a_box_hull_under_a_sphere();
    b3::LocalManifoldPoint deep = pair.centre_inside.points[0];

    assert(pair.centre_inside.point_count == 1, "a centre inside the hull still reports a point");
    assert(0.9999 < pair.centre_inside.normal.y && pair.centre_inside.normal.y < 1.0001,
        "the face the centre is least deep against is the one the normal comes from");

    float expected_separation = (0.2f - 0.5f) - 0.5f;
    assert(-0.8001 < deep.separation && deep.separation < -0.7999,
        "and the separation is measured to that face plane, not to the surface the centre passed");
    assert(math::abs(deep.separation - expected_separation) < 0.00001);
}

struct CapsulePairReadings {
    ManifoldReading parallel;
    ManifoldReading crossing;
    ManifoldReading a_too_short;
    ManifoldReading b_too_short;
    ManifoldReading a_short_enough;
}

// Two metre-long capsules laid parallel 0.4 apart, against a summed radius of 0.5. The short
// capsules bracket box3d's minimum length of one linear slop, 0.005 at unit length scale: 0.001 is
// well under it and 0.02 well over, and both overlap the other capsule by the same 0.1.
fn CapsulePairReadings the_readings_of_two_parallel_capsules() {
    b3::Capsule a = a_metre_capsule_along_x();
    b3::Capsule b = a_metre_capsule_along_x();
    b3::Capsule along_z = { .center1 = { 0.0, 0.0, -0.5 }, .center2 = { 0.0, 0.0, 0.5 }, .radius = 0.25 };
    b3::Capsule too_short = { .center1 = { 0.0, 0.0, 0.0 }, .center2 = { 0.001, 0.0, 0.0 }, .radius = 0.25 };
    b3::Capsule short_enough = { .center1 = { 0.0, 0.0, 0.0 }, .center2 = { 0.02, 0.0, 0.0 }, .radius = 0.25 };
    b3::Transform above = a_placement_at({ 0.0, 0.4, 0.0 });

    return {
      .parallel = the_reading_of(b3::collide_capsules(&a, b, above)),
      .crossing = the_reading_of(b3::collide_capsules(&a, along_z, a_placement_at({ 0.0, 0.0, 0.0 }))),
      .a_too_short = the_reading_of(b3::collide_capsules(&too_short, b, above)),
      .b_too_short = the_reading_of(b3::collide_capsules(&a, too_short, above)),
      .a_short_enough = the_reading_of(b3::collide_capsules(&short_enough, b, above)),
    };
}

fn void two_parallel_capsules_report_a_point_at_each_end() @test {
    CapsulePairReadings capsules = the_readings_of_two_parallel_capsules();

    assert(capsules.parallel.point_count == 2, "two parallel capsules meet along a segment, not at a point");
    assert(capsules.parallel.slice_len == 2);

    b3::LocalManifoldPoint first = capsules.parallel.points[0];
    b3::LocalManifoldPoint second = capsules.parallel.points[1];

    float expected_separation = 0.4f - (0.25f + 0.25f);
    assert(-0.1001 < first.separation && first.separation < -0.0999,
        "both ends overlap by the same amount, the axis gap less the two radii");
    assert(math::abs(first.separation - expected_separation) < 0.00001);
    assert(math::abs(second.separation - expected_separation) < 0.00001);

    float expected_y = 0.5f * ((0.4f - 0.25f) + 0.25f);
    assert(math::abs(first.point.y - expected_y) < 0.00001, "and both sit half way between the two surfaces");
    assert(math::abs(second.point.y - expected_y) < 0.00001);
    assert(math::abs(first.point.x + 0.5f) < 0.00001, "one at each end of the overlapping span");
    assert(math::abs(second.point.x - 0.5f) < 0.00001);

    assert(first.pair.index1 != second.pair.index1,
        "the two points name different features, which is how box3d matches them across steps");

    assert(0.9999 < capsules.parallel.normal.y && capsules.parallel.normal.y < 1.0001,
        "the normal runs from this capsule towards the other");
    assert(math::abs(the_length_of(capsules.parallel.normal) - 1.0f) < 0.00001);
}

fn void a_capsule_shorter_than_a_linear_slop_reports_no_contact_at_all() @test {
    CapsulePairReadings capsules = the_readings_of_two_parallel_capsules();

    assert(capsules.a_short_enough.point_count == 2,
        "a capsule four linear slops long meets the one it overlaps");
    assert(capsules.a_too_short.point_count == 0,
        "a fifth of a slop long, the same overlap reports nothing, this capsule being one box3d declines");
    assert(capsules.b_too_short.point_count == 0, "and it declines on either side of the pair");

    assert(math::abs(capsules.a_short_enough.points[0].separation + 0.1f) < 0.00001,
        "the short capsule that is collided overlaps by exactly what the long one does");
}

fn void two_capsules_whose_axes_exactly_cross_report_nothing() @test {
    CapsulePairReadings capsules = the_readings_of_two_parallel_capsules();

    assert(capsules.crossing.point_count == 0,
        "axes meeting at a point leave box3d no direction to separate along, however deep the overlap");
    assert(the_length_of(capsules.crossing.normal) < 0.00001, "so no normal comes back either");
}

fn void collide_calls_allocate_nothing() @test {
    int before = b3::byte_count();

    the_readings_of_two_half_metre_spheres();
    the_readings_of_a_capsule_under_a_sphere();
    the_readings_of_a_box_hull_under_a_sphere();
    the_readings_of_two_parallel_capsules();
    the_readings_of_a_box_hull_under_a_capsule();
    the_readings_of_two_box_hulls();
    the_readings_of_a_triangle_under_a_sphere();
    the_readings_of_a_triangle_under_a_capsule();
    the_refusals_of_a_triangle_and_hull_call();

    assert(b3::byte_count() == before,
        "every manifold is built on the caller's own stack, so box3d's byte count does not move");
}

fn b3::Capsule a_short_capsule_along_x() {
    return { .center1 = { -0.3, 0.0, 0.0 }, .center2 = { 0.3, 0.0, 0.0 }, .radius = 0.25 };
}

struct HullAndCapsuleReadings {
    ManifoldReading overlapping;
    ManifoldReading just_clear;
    ManifoldReading apart;
    ManifoldReading one_end_lifted;
}

// The capsule lies along the box's top face, its axis 0.2 above a face at y 0.5 against a radius
// of 0.25. The lifted capsule keeps that end where it was and raises the other a metre, which is
// far outside anything the pair could call contact.
fn HullAndCapsuleReadings the_readings_of_a_box_hull_under_a_capsule() {
    b3::BoxHull box = b3::make_box_hull(0.5, 0.5, 0.5);
    b3::Capsule capsule = a_short_capsule_along_x();
    b3::Capsule lifted = { .center1 = { -0.3, 0.0, 0.0 }, .center2 = { 0.3, 1.0, 0.0 }, .radius = 0.25 };

    return {
      .overlapping = the_reading_of(b3::collide_hull_and_capsule(&box.base, capsule, a_placement_at({ 0.0, 0.7, 0.0 }))),
      .just_clear = the_reading_of(b3::collide_hull_and_capsule(&box.base, capsule, a_placement_at({ 0.0, 0.76, 0.0 }))),
      .apart = the_reading_of(b3::collide_hull_and_capsule(&box.base, capsule, a_placement_at({ 0.0, 0.78, 0.0 }))),
      .one_end_lifted = the_reading_of(b3::collide_hull_and_capsule(&box.base, lifted, a_placement_at({ 0.0, 0.7, 0.0 }))),
    };
}

fn void a_hull_and_a_capsule_report_a_point_at_each_end_of_the_segment() @test {
    HullAndCapsuleReadings pair = the_readings_of_a_box_hull_under_a_capsule();

    assert(pair.overlapping.point_count == 2, "a capsule lying along a face meets it along a segment");
    assert(pair.overlapping.slice_len == 2);

    b3::LocalManifoldPoint first = pair.overlapping.points[0];
    b3::LocalManifoldPoint second = pair.overlapping.points[1];

    float expected_separation = (0.7f - 0.5f) - 0.25f;
    assert(-0.0501 < first.separation && first.separation < -0.0499,
        "both ends overlap by the axis height above the face less the capsule's radius");
    assert(math::abs(first.separation - expected_separation) < 0.00001);
    assert(math::abs(second.separation - expected_separation) < 0.00001);

    float expected_y = 0.7f - 0.5f * (0.2f + 0.25f);
    assert(math::abs(first.point.y - expected_y) < 0.00001, "and both sit half way between the two surfaces");
    assert(math::abs(second.point.y - expected_y) < 0.00001);
    assert(math::abs(first.point.x + 0.3f) < 0.00001, "one at each end of the capsule's segment");
    assert(math::abs(second.point.x - 0.3f) < 0.00001);

    assert(0.9999 < pair.overlapping.normal.y && pair.overlapping.normal.y < 1.0001,
        "the normal runs from the hull towards the capsule");
    assert(math::abs(the_length_of(pair.overlapping.normal) - 1.0f) < 0.00001);
}

fn void a_hull_and_a_capsule_report_daylight_only_inside_the_window() @test {
    HullAndCapsuleReadings pair = the_readings_of_a_box_hull_under_a_capsule();

    assert(pair.just_clear.point_count == 2,
        "a hundredth of a metre of daylight is inside this pair's speculative window");
    assert(0.0099 < pair.just_clear.points[0].separation && pair.just_clear.points[0].separation < 0.0101,
        "and what it reports is positive, being real daylight rather than an overlap");

    assert(pair.apart.point_count == 0, "three hundredths is past the window and meets nothing");
    assert(pair.apart.slice_len == 0);
    assert(the_length_of(pair.apart.normal) < 0.00001,
        "a miss read after a contact carries no normal either");
}

fn void a_capsule_with_one_end_lifted_clear_still_reports_that_end() @test {
    HullAndCapsuleReadings pair = the_readings_of_a_box_hull_under_a_capsule();

    assert(pair.one_end_lifted.point_count == 2,
        "box3d admits the pair on the end that touches and then reports both ends of the segment");
    assert(math::abs(pair.one_end_lifted.points[0].separation + 0.05f) < 0.00001,
        "the resting end overlaps exactly as the level capsule's does");
    assert(0.9499 < pair.one_end_lifted.points[1].separation && pair.one_end_lifted.points[1].separation < 0.9501,
        "while the other is most of a metre clear of the face it is reported against");
}

struct HullPairReadings {
    ManifoldReading overlapping;
    ManifoldReading touching;
    ManifoldReading just_clear;
    ManifoldReading apart;
    fault null_hull;
}

// Two half-metre box hulls face to face along x, the second overlapping the first by 0.1, then
// exactly meeting it, then a hundredth clear, then three metres away.
fn HullPairReadings the_readings_of_two_box_hulls() {
    b3::BoxHull a = b3::make_box_hull(0.5, 0.5, 0.5);
    b3::BoxHull b = b3::make_box_hull(0.5, 0.5, 0.5);
    HullPairReadings readings = {
      .overlapping = the_reading_of(b3::collide_hulls(&a.base, &b.base, a_placement_at({ 0.9, 0.0, 0.0 }))!!),
      .touching = the_reading_of(b3::collide_hulls(&a.base, &b.base, a_placement_at({ 1.0, 0.0, 0.0 }))!!),
      .just_clear = the_reading_of(b3::collide_hulls(&a.base, &b.base, a_placement_at({ 1.01, 0.0, 0.0 }))!!),
      .apart = the_reading_of(b3::collide_hulls(&a.base, &b.base, a_placement_at({ 3.0, 0.0, 0.0 }))!!),
    };

    if (catch err = b3::collide_hulls(&a.base, null, a_placement_at({ 0.9, 0.0, 0.0 }))) readings.null_hull = err;
    return readings;
}

fn void two_hulls_report_the_four_corners_of_the_face_they_share() @test {
    HullPairReadings hulls = the_readings_of_two_box_hulls();

    assert(hulls.overlapping.point_count == 4,
        "two boxes meet across a face, and box3d reduces that face to its own manifold size");
    assert(hulls.overlapping.slice_len == 4);

    foreach (point : hulls.overlapping.points[:hulls.overlapping.point_count]) {
        assert(math::abs(point.separation + 0.1f) < 0.00001,
            "every corner overlaps by the same tenth of a metre");
        assert(math::abs(point.point.x - 0.45f) < 0.00001, "and sits half way through that overlap");
        assert(math::abs(math::abs(point.point.y) - 0.5f) < 0.00001, "at a corner of the shared face");
        assert(math::abs(math::abs(point.point.z) - 0.5f) < 0.00001);
    }

    assert(0.9999 < hulls.overlapping.normal.x && hulls.overlapping.normal.x < 1.0001,
        "the normal runs from this hull towards the other");
    assert(math::abs(the_length_of(hulls.overlapping.normal) - 1.0f) < 0.00001);
}

fn void a_hull_pair_separation_reaches_zero_and_then_daylight() @test {
    HullPairReadings hulls = the_readings_of_two_box_hulls();

    assert(hulls.touching.point_count == 4, "two faces exactly meeting still report the whole face");
    assert(math::abs(hulls.touching.points[0].separation) < 0.0000001,
        "and the separation there is zero rather than negative, the separating test being strict");

    assert(hulls.just_clear.point_count == 4,
        "a hundredth of a metre of daylight is inside this pair's speculative window");
    assert(0.0099 < hulls.just_clear.points[0].separation && hulls.just_clear.points[0].separation < 0.0101);

    assert(hulls.apart.point_count == 0, "three metres away they meet nowhere");
    assert(the_length_of(hulls.apart.normal) < 0.00001);
}

fn void a_hull_pair_refuses_a_null_second_hull() @test {
    HullPairReadings hulls = the_readings_of_two_box_hulls();

    assert(hulls.overlapping.point_count == 4, "the same call against a real hull answers four points");
    assert(hulls.null_hull == b3::INVALID_DEFINITION,
        "a null one is refused rather than read, box3d reaching its edge array before testing anything");
}

// Wound so its plane normal is +y: the cross product of the second and third vertices about the
// first runs upward, which is the side a shape has to be on to be reported at all.
fn b3::Triangle a_wide_triangle_facing_up() {
    return { { -2.0, 0.0, -2.0 }, { 0.0, 0.0, 3.0 }, { 2.0, 0.0, -2.0 } };
}

struct TriangleAndSphereReadings {
    ManifoldReading overlapping;
    ManifoldReading touching;
    ManifoldReading just_clear;
    ManifoldReading apart;
    ManifoldReading behind;
}

// A half-metre sphere lowered onto a triangle in the xz plane, and the same overlap read from the
// culled side, where the sphere is as deep as it is in the first reading.
fn TriangleAndSphereReadings the_readings_of_a_triangle_under_a_sphere() {
    b3::Triangle triangle = a_wide_triangle_facing_up();
    b3::Sphere overlapping = { .center = { 0.0, 0.4, 0.0 }, .radius = 0.5 };
    b3::Sphere touching = { .center = { 0.0, 0.5, 0.0 }, .radius = 0.5 };
    b3::Sphere just_clear = { .center = { 0.0, 0.52, 0.0 }, .radius = 0.5 };
    b3::Sphere apart = { .center = { 0.0, 0.53, 0.0 }, .radius = 0.5 };
    b3::Sphere behind = { .center = { 0.0, -0.4, 0.0 }, .radius = 0.5 };

    return {
      .overlapping = the_reading_of(b3::collide_triangle_and_sphere(&triangle, overlapping)!!),
      .touching = the_reading_of(b3::collide_triangle_and_sphere(&triangle, touching)!!),
      .just_clear = the_reading_of(b3::collide_triangle_and_sphere(&triangle, just_clear)!!),
      .apart = the_reading_of(b3::collide_triangle_and_sphere(&triangle, apart)!!),
      .behind = the_reading_of(b3::collide_triangle_and_sphere(&triangle, behind)!!),
    };
}

fn void a_triangle_and_a_sphere_report_the_point_between_them() @test {
    TriangleAndSphereReadings pair = the_readings_of_a_triangle_under_a_sphere();

    assert(pair.overlapping.point_count == 1, "a triangle and a sphere meet at one point");
    assert(pair.overlapping.slice_len == 1);

    b3::LocalManifoldPoint contact = pair.overlapping.points[0];
    float expected_separation = 0.4f - 0.5f;
    assert(-0.1001 < contact.separation && contact.separation < -0.0999,
        "the separation is the centre's height above the plane less the sphere's radius");
    assert(math::abs(contact.separation - expected_separation) < 0.00001);

    float expected_y = 0.5f * (0.4f - 0.5f);
    assert(math::abs(contact.point.y - expected_y) < 0.00001,
        "and the point sits half way between the plane and the sphere's deepest point");
    assert(math::abs(contact.point.x) < 0.00001 && math::abs(contact.point.z) < 0.00001);

    assert(0.9999 < pair.overlapping.normal.y && pair.overlapping.normal.y < 1.0001,
        "the normal runs from the triangle towards the sphere");
    assert(math::abs(the_length_of(pair.overlapping.normal) - 1.0f) < 0.00001);
}

fn void a_triangle_and_sphere_separation_reaches_zero_and_then_daylight() @test {
    TriangleAndSphereReadings pair = the_readings_of_a_triangle_under_a_sphere();

    assert(pair.touching.point_count == 1, "a sphere resting exactly on the plane still reports");
    assert(math::abs(pair.touching.points[0].separation) < 0.0000001,
        "and its separation is zero rather than negative, the separating test being strict");

    assert(pair.just_clear.point_count == 1,
        "two hundredths of a metre of daylight is the far edge of the speculative window");
    assert(0.0199 < pair.just_clear.points[0].separation && pair.just_clear.points[0].separation < 0.0201);

    assert(pair.apart.point_count == 0, "a hundredth further out meets nothing");
    assert(the_length_of(pair.apart.normal) < 0.00001);
}

fn void a_sphere_behind_a_triangle_meets_nothing_however_deep_it_is() @test {
    TriangleAndSphereReadings pair = the_readings_of_a_triangle_under_a_sphere();

    assert(pair.overlapping.point_count == 1,
        "the sphere a tenth of a metre into the front of the triangle reports a point");
    assert(pair.behind.point_count == 0,
        "the same overlap on the other side reports none: the winding decides which side answers");
    assert(the_length_of(pair.behind.normal) < 0.00001);
}

struct TriangleAndCapsuleReadings {
    ManifoldReading overlapping;
    ManifoldReading just_clear;
    ManifoldReading apart;
    ManifoldReading behind;
    ManifoldReading midpoint_behind;
}

// The capsule lies along the triangle's plane 0.2 above it against a radius of 0.25. The last
// reading tips it until one end is 0.2 in front of the plane and the midpoint of its two centres
// is 0.1 behind, which is what the back-side cull actually reads.
fn TriangleAndCapsuleReadings the_readings_of_a_triangle_under_a_capsule() {
    b3::Triangle triangle = a_wide_triangle_facing_up();
    b3::Capsule overlapping = { .center1 = { -0.3, 0.2, 0.0 }, .center2 = { 0.3, 0.2, 0.0 }, .radius = 0.25 };
    b3::Capsule just_clear = { .center1 = { -0.3, 0.27, 0.0 }, .center2 = { 0.3, 0.27, 0.0 }, .radius = 0.25 };
    b3::Capsule apart = { .center1 = { -0.3, 0.28, 0.0 }, .center2 = { 0.3, 0.28, 0.0 }, .radius = 0.25 };
    b3::Capsule behind = { .center1 = { -0.3, -0.2, 0.0 }, .center2 = { 0.3, -0.2, 0.0 }, .radius = 0.25 };
    b3::Capsule tipped = { .center1 = { -0.3, 0.2, 0.0 }, .center2 = { 0.3, -0.4, 0.0 }, .radius = 0.25 };

    return {
      .overlapping = the_reading_of(b3::collide_triangle_and_capsule(&triangle, overlapping)!!),
      .just_clear = the_reading_of(b3::collide_triangle_and_capsule(&triangle, just_clear)!!),
      .apart = the_reading_of(b3::collide_triangle_and_capsule(&triangle, apart)!!),
      .behind = the_reading_of(b3::collide_triangle_and_capsule(&triangle, behind)!!),
      .midpoint_behind = the_reading_of(b3::collide_triangle_and_capsule(&triangle, tipped)!!),
    };
}

fn void a_triangle_and_a_capsule_report_a_point_at_each_end_of_the_segment() @test {
    TriangleAndCapsuleReadings pair = the_readings_of_a_triangle_under_a_capsule();

    assert(pair.overlapping.point_count == 2, "a capsule lying along the plane meets it along a segment");
    assert(pair.overlapping.slice_len == 2);

    b3::LocalManifoldPoint first = pair.overlapping.points[0];
    b3::LocalManifoldPoint second = pair.overlapping.points[1];

    float expected_separation = 0.2f - 0.25f;
    assert(-0.0501 < first.separation && first.separation < -0.0499,
        "both ends overlap by the axis height above the plane less the capsule's radius");
    assert(math::abs(first.separation - expected_separation) < 0.00001);
    assert(math::abs(second.separation - expected_separation) < 0.00001);

    float expected_y = 0.2f - 0.5f * (0.2f + 0.25f);
    assert(math::abs(first.point.y - expected_y) < 0.00001, "and both sit half way between the two surfaces");
    assert(math::abs(second.point.y - expected_y) < 0.00001);
    assert(math::abs(first.point.x + 0.3f) < 0.00001, "one at each end of the capsule's segment");
    assert(math::abs(second.point.x - 0.3f) < 0.00001);

    assert(0.9999 < pair.overlapping.normal.y && pair.overlapping.normal.y < 1.0001,
        "the normal runs from the triangle towards the capsule");
    assert(math::abs(the_length_of(pair.overlapping.normal) - 1.0f) < 0.00001);
}

fn void a_triangle_and_a_capsule_report_daylight_only_inside_the_window() @test {
    TriangleAndCapsuleReadings pair = the_readings_of_a_triangle_under_a_capsule();

    assert(pair.just_clear.point_count == 2,
        "two hundredths of a metre of daylight is the far edge of the speculative window");
    assert(0.0199 < pair.just_clear.points[0].separation && pair.just_clear.points[0].separation < 0.0201,
        "and what it reports is positive, being real daylight rather than an overlap");

    assert(pair.apart.point_count == 0, "a hundredth further out meets nothing");
    assert(the_length_of(pair.apart.normal) < 0.00001);
}

fn void a_capsule_is_culled_on_the_midpoint_of_its_centres_not_on_its_nearer_end() @test {
    TriangleAndCapsuleReadings pair = the_readings_of_a_triangle_under_a_capsule();

    assert(pair.overlapping.point_count == 2, "the capsule lying in front of the plane reports both ends");
    assert(pair.behind.point_count == 0, "the same overlap behind it reports nothing");
    assert(pair.midpoint_behind.point_count == 0,
        "and so does a capsule with one end a fifth of a metre in front, its midpoint being behind");
    assert(the_length_of(pair.midpoint_behind.normal) < 0.00001);
}

const usz CLIPPED_POINT_SLOTS = 8;

// What one triangle-and-hull call answered, copied out of the buffer the view borrows so the
// reading outlives it. count comes from the returned slice, which is the only bound the pair has.
struct ClippedReading {
    usz count;
    b3::Vec3 normal;
    b3::LocalManifoldPoint[CLIPPED_POINT_SLOTS] points;
}

fn ClippedReading the_clipped_reading_of(
  b3::Triangle triangle,
  b3::HullData* hull,
  bool enable_speculative,
  usz slots,
) {
    b3::LocalManifoldPoint[CLIPPED_POINT_SLOTS] buffer;
    b3::CollisionManifoldView view = b3::collide_triangle_and_hull(&triangle, hull, enable_speculative, buffer[:slots])!!;

    ClippedReading reading = { .count = view.points.len, .normal = view.normal };
    foreach (index, point : view.points) {
        reading.points[index] = point;
    }
    return reading;
}

fn usz the_points_reaching_the_far_edge(ClippedReading reading) {
    usz reached = 0;
    foreach (point : reading.points[:reading.count]) {
        if (point.point.x < -0.4999f) reached += 1;
    }
    return reached;
}

// Wound so its plane normal is -y, which is the side the box hull at the origin sits on.
fn b3::Triangle a_wide_triangle_facing_down_at(float height) {
    return { { -2.0, height, -2.0 }, { 2.0, height, -2.0 }, { 0.0, height, 3.0 } };
}

// An equilateral triangle laid across the box's top face off centre and turned ten degrees, so
// the face clips it into a seven-sided polygon rather than into the face itself.
fn b3::Triangle a_triangle_clipping_a_box_face_at_seven_points() {
    return {
      { 0.589369, 0.4, 0.121531 },
      { -0.549934, 0.4, 0.536246 },
      { -0.339436, 0.4, -0.657777 },
    };
}

fn void a_triangle_and_a_hull_report_the_face_they_share() @test {
    b3::BoxHull box = b3::make_box_hull(0.5, 0.5, 0.5);
    ClippedReading contact = the_clipped_reading_of(
      triangle: a_wide_triangle_facing_down_at(0.4),
      hull: &box.base,
      enable_speculative: true,
      slots: CLIPPED_POINT_SLOTS,
    );

    assert(contact.count == 4, "a triangle across the whole of a box's face meets it at that face's corners");
    foreach (point : contact.points[:contact.count]) {
        assert(math::abs(point.separation + 0.1f) < 0.00001,
            "every corner is the same tenth of a metre through the triangle's plane");
        assert(math::abs(point.point.y - 0.5f) < 0.00001, "and lies on the face itself");
        assert(math::abs(math::abs(point.point.x) - 0.5f) < 0.00001);
        assert(math::abs(math::abs(point.point.z) - 0.5f) < 0.00001);
    }

    assert(-1.0001 < contact.normal.y && contact.normal.y < -0.9999,
        "the normal runs from the triangle towards the hull, which is below it");
    assert(math::abs(the_length_of(contact.normal) - 1.0f) < 0.00001);
}

fn void a_hull_behind_a_triangle_meets_nothing() @test {
    b3::BoxHull box = b3::make_box_hull(0.5, 0.5, 0.5);
    ClippedReading anchored = the_clipped_reading_of(
      triangle: a_wide_triangle_facing_down_at(0.4),
      hull: &box.base,
      enable_speculative: true,
      slots: CLIPPED_POINT_SLOTS,
    );
    ClippedReading culled = the_clipped_reading_of(
      triangle: a_wide_triangle_facing_down_at(-0.6),
      hull: &box.base,
      enable_speculative: true,
      slots: CLIPPED_POINT_SLOTS,
    );

    assert(anchored.count == 4, "the triangle above the hull meets its top face");
    assert(culled.count == 0, "the same triangle below it, its plane normal pointing away, meets nothing");
    assert(the_length_of(culled.normal) < 0.00001);
}

fn void the_speculative_flag_decides_whether_daylight_is_reported() @test {
    b3::BoxHull box = b3::make_box_hull(0.5, 0.5, 0.5);
    ClippedReading speculating = the_clipped_reading_of(
      triangle: a_wide_triangle_facing_down_at(0.51),
      hull: &box.base,
      enable_speculative: true,
      slots: CLIPPED_POINT_SLOTS,
    );
    ClippedReading touching_only = the_clipped_reading_of(
      triangle: a_wide_triangle_facing_down_at(0.51),
      hull: &box.base,
      enable_speculative: false,
      slots: CLIPPED_POINT_SLOTS,
    );

    assert(speculating.count == 4, "a hundredth of a metre of daylight is inside the speculative window");
    assert(0.0099 < speculating.points[0].separation && speculating.points[0].separation < 0.0101,
        "and what it reports is positive, being real daylight rather than an overlap");

    assert(touching_only.count == 0, "with the window switched off the same gap meets nothing");
    assert(the_length_of(touching_only.normal) < 0.00001);
}

fn void the_buffer_bounds_a_triangle_and_hull_manifold_rather_than_reducing_it() @test {
    b3::BoxHull box = b3::make_box_hull(0.5, 0.5, 0.5);
    b3::Triangle triangle = a_triangle_clipping_a_box_face_at_seven_points();

    ClippedReading roomy = the_clipped_reading_of(
      triangle: triangle,
      hull: &box.base,
      enable_speculative: true,
      slots: CLIPPED_POINT_SLOTS,
    );
    ClippedReading cramped = the_clipped_reading_of(
      triangle: triangle,
      hull: &box.base,
      enable_speculative: true,
      slots: (usz)b3::MAX_MANIFOLD_POINTS,
    );

    assert(roomy.count == 7, "this triangle cuts the box's face into a seven-sided polygon");
    foreach (point : roomy.points[:roomy.count]) {
        assert(math::abs(point.separation + 0.1f) < 0.00001, "every one of whose corners is the same depth");
        assert(math::abs(point.point.y - 0.5f) < 0.00001);
    }

    assert(cramped.count == (usz)b3::MAX_MANIFOLD_POINTS,
        "a buffer of box3d's own manifold size keeps four of those seven");
    assert(the_points_reaching_the_far_edge(roomy) == 2,
        "two of the seven sit against the face edge the short buffer stops before");
    assert(the_points_reaching_the_far_edge(cramped) == 0,
        "so what four points buy is a prefix of the clipped polygon, not a reduction of it");
}

struct ClippedRefusals {
    fault null_hull;
    fault short_buffer;
    usz answered;
}

fn ClippedRefusals the_refusals_of_a_triangle_and_hull_call() {
    b3::BoxHull box = b3::make_box_hull(0.5, 0.5, 0.5);
    b3::Triangle triangle = a_wide_triangle_facing_down_at(0.4);
    b3::LocalManifoldPoint[CLIPPED_POINT_SLOTS] buffer;
    ClippedRefusals refusals;

    if (catch err = b3::collide_triangle_and_hull(&triangle, null, true, buffer[..])) refusals.null_hull = err;
    if (catch err = b3::collide_triangle_and_hull(&triangle, &box.base, true, buffer[:(usz)b3::MAX_MANIFOLD_POINTS - 1])) {
        refusals.short_buffer = err;
    }

    b3::CollisionManifoldView answered = b3::collide_triangle_and_hull(&triangle, &box.base, true, buffer[..])!!;
    refusals.answered = answered.points.len;
    return refusals;
}

fn void a_triangle_and_hull_call_refuses_what_box3d_would_answer_as_no_contact() @test {
    ClippedRefusals refusals = the_refusals_of_a_triangle_and_hull_call();

    assert(refusals.answered == 4, "the same call with a hull and room to write reports four points");
    assert(refusals.null_hull == b3::INVALID_DEFINITION,
        "a null hull is refused rather than read, box3d reaching its edge array before testing anything");
    assert(refusals.short_buffer == b3::BUFFER_TOO_SMALL,
        "and a buffer under box3d's floor is refused, where box3d would report no contact at all");
}

// box3d holds a back-side cull in its warm start and then declines a second call whose offset is
// within a linear slop of the cached one. These two heights are 0.002 apart and straddle the slop,
// so a warm start carried between them would turn the second call's real contact into no contact.
fn void a_triangle_and_hull_call_never_inherits_the_previous_call_axis() @test {
    b3::BoxHull box = b3::make_box_hull(0.5, 0.5, 0.5);
    b3::Triangle over_the_centre = { { -2.0, 0.006, -2.0 }, { 0.0, 0.006, 3.0 }, { 2.0, 0.006, -2.0 } };
    b3::Triangle just_under_it = { { -2.0, 0.004, -2.0 }, { 0.0, 0.004, 3.0 }, { 2.0, 0.004, -2.0 } };

    ClippedReading culled = the_clipped_reading_of(
      triangle: over_the_centre,
      hull: &box.base,
      enable_speculative: true,
      slots: CLIPPED_POINT_SLOTS,
    );
    ClippedReading across = the_clipped_reading_of(
      triangle: just_under_it,
      hull: &box.base,
      enable_speculative: true,
      slots: CLIPPED_POINT_SLOTS,
    );

    assert(culled.count == 0, "the hull's centre sits a slop behind this triangle, so it is culled");
    assert(across.count == 4,
        "two thousandths lower the same triangle cuts the hull and reports its far face");
    assert(math::abs(across.points[0].separation + 0.504f) < 0.001,
        "at the depth that face lies below the plane, not at anything the culled call left behind");
}

struct DegenerateTriangleRefusals {
    fault sphere;
    fault capsule;
    fault hull;
    int anchor_points;
}

fn DegenerateTriangleRefusals the_refusals_of_a_triangle_with_no_area() {
    // Two coincident vertices: box3d's plane normal falls under its own degeneracy floor, so it
    // culls nothing and the closest-point solve divides by a zero barycentric sum.
    b3::Triangle flat = { { -1.0, 0.0, 0.0 }, { -1.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 } };
    b3::Sphere ball = { .center = { 0.0, 0.2, 0.0 }, .radius = 0.5 };
    b3::Capsule rod = { .center1 = { -0.3, 0.2, 0.0 }, .center2 = { 0.3, 0.2, 0.0 }, .radius = 0.25 };
    b3::BoxHull cube = b3::make_cube_hull(0.5);
    b3::LocalManifoldPoint[8] buffer;

    DegenerateTriangleRefusals refused;
    if (catch err = b3::collide_triangle_and_sphere(&flat, ball)) refused.sphere = err;
    if (catch err = b3::collide_triangle_and_capsule(&flat, rod)) refused.capsule = err;
    if (catch err = b3::collide_triangle_and_hull(&flat, &cube.base, true, buffer[..])) refused.hull = err;

    b3::Triangle real = a_wide_triangle_facing_up();
    if (try contact = b3::collide_triangle_and_sphere(&real, ball)) refused.anchor_points = contact.point_count;

    return refused;
}

// A triangle whose vertices name no area has no plane, so box3d culls nothing and answers the
// sphere pair with a NaN point. Refused before it reaches a caller's solver, where a NaN spreads.
fn void a_triangle_with_no_area_is_refused_by_every_pair_that_takes_one() @test {
    DegenerateTriangleRefusals refused = the_refusals_of_a_triangle_with_no_area();

    assert(refused.sphere == b3::DEGENERATE_GEOMETRY, "a triangle of no area names no plane to cull against");
    assert(refused.capsule == b3::DEGENERATE_GEOMETRY, "and the capsule pair reads the same plane");
    assert(refused.hull == b3::DEGENERATE_GEOMETRY, "and so does the hull pair, before its buffer check");
    assert(refused.anchor_points == 1, "while a triangle with area still answers the same sphere");
}
