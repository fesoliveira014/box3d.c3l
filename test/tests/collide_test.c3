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
      .overlapping = the_reading_of(a.collide_sphere(b, a_placement_at({ 0.75, 0.0, 0.0 }))),
      .apart = the_reading_of(a.collide_sphere(b, a_placement_at({ 3.0, 0.0, 0.0 }))),
      .just_clear = the_reading_of(a.collide_sphere(b, a_placement_at({ 1.01, 0.0, 0.0 }))),
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
      .overlapping = the_reading_of(capsule.collide_sphere(sphere, a_placement_at({ 0.0, 0.6, 0.0 }))),
      .just_clear = the_reading_of(capsule.collide_sphere(sphere, a_placement_at({ 0.0, 0.76, 0.0 }))),
      .centres_coincident = the_reading_of(collapsed.collide_sphere(sphere, a_placement_at({ 0.0, 0.6, 0.0 }))),
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
      .overlapping = the_reading_of(box.base.collide_sphere(sphere, a_placement_at({ 0.0, 0.9, 0.0 }))),
      .just_clear = the_reading_of(box.base.collide_sphere(sphere, a_placement_at({ 0.0, 1.01, 0.0 }))),
      .centre_inside = the_reading_of(box.base.collide_sphere(sphere, a_placement_at({ 0.0, 0.2, 0.0 }))),
      .apart = the_reading_of(box.base.collide_sphere(sphere, a_placement_at({ 0.0, 3.0, 0.0 }))),
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
      .parallel = the_reading_of(a.collide_capsule(b, above)),
      .crossing = the_reading_of(a.collide_capsule(along_z, a_placement_at({ 0.0, 0.0, 0.0 }))),
      .a_too_short = the_reading_of(too_short.collide_capsule(b, above)),
      .b_too_short = the_reading_of(a.collide_capsule(too_short, above)),
      .a_short_enough = the_reading_of(short_enough.collide_capsule(b, above)),
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

    assert(b3::byte_count() == before,
        "every manifold is built on the caller's own stack, so box3d's byte count does not move");
}
