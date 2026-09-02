module b3;

// box3d exports its unit-vector test only inside b3IsValidPlane, whose plane is valid exactly when
// the normal is finite and normalised, so this borrows box3d's own tolerance rather than mirroring
// the constant it is written with.
fn bool is_unit_vector(Vec3 v) @private {
    Plane plane = { .normal = v, .offset = 0.0 };
    return is_valid_plane(plane);
}

// The quantity box3d divides by wherever two lines are parallel, which a zero direction always is.
// Summed the way box3d sums it, so a direction whose square underflows to zero is caught here too.
fn bool has_length(Vec3 v) @private {
    return v.x * v.x + v.y * v.y + v.z * v.z > 0.0f;
}

<*
 The rotation the matrix's three columns describe, normalised. cx, cy and cz are columns: a caller
 filling them from rows gets the inverse rotation, which no layout pin can catch.
*>
fn Quat make_quat_from_matrix(Matrix3 m) {
    return make_quat_from_matrix_raw(&m);
}

<*
 The shortest rotation taking v1 onto v2. Antipodal vectors have no shortest rotation, so box3d
 answers a half turn about an axis it picks perpendicular to v1 rather than an identity.

 @return "the rotation, or NOT_A_UNIT_VECTOR when either vector is not one — box3d asserts both and this release build compiles that out, leaving an unequally scaled pair reading as a different angle and a pair shorter than about 1e-5 as no rotation at all"
*>
fn Quat? compute_quat_between_unit_vectors(Vec3 v1, Vec3 v2) {
    if (!is_unit_vector(v1) || !is_unit_vector(v2)) return NOT_A_UNIT_VECTOR~;
    return compute_quat_between_unit_vectors_raw(v1, v2);
}

<*
 The closest points on two infinite lines, each named by a point and a direction — not by two
 endpoints, which is what segment_distance takes. The fractions run along the directions as given
 and are unbounded, so a closest point behind a line's own origin reports a negative one.

 @return "the closest points, or DEGENERATE_GEOMETRY when d1 has no length, which box3d divides by and answers a NaN point for. A zero d2 is answered rather than refused: it reports p2 itself and the point on the first line nearest it"
*>
fn SegmentDistanceResult? line_distance(Vec3 p1, Vec3 d1, Vec3 p2, Vec3 d2) {
    if (!has_length(d1)) return DEGENERATE_GEOMETRY~;
    return line_distance_raw(
      p1: p1,
      d1: d1,
      p2: p2,
      d2: d2,
    );
}
