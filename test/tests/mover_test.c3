module b3_test;

import b3;
import std::math;

// A push no solve of these fixtures produces, so a plane handed in carrying it and read back at
// anything else proves the solve rewrites the push rather than accumulating onto it.
const float A_PUSH_NO_SOLVE_PRODUCES = 123.0;

// How far below the ground plane the opposing plane of the last fixture demands the mover sit,
// which is what makes that pair impossible to satisfy at once.
const float AN_OPPOSING_OFFSET = 0.5;

// What one plane solve left behind: its own result, and the pushes it wrote back through the
// caller's array. plane_count is carried so a test can assert it before naming either push, and
// converged_on_the_first_pass so a converging first pass does not read as a reading left at zero.
struct PlaneSolve {
    b3::Vec3 delta;
    int iteration_count;
    bool converged_on_the_first_pass;
    int plane_count;
    float first_push;
    float second_push;
}

// A rigid clipping plane through the origin, facing whichever way it is asked to. Every fixture
// below starts from one of these, so a reading that differs does so for the reason its own fixture
// names rather than for a difference in the plane.
fn b3::CollisionPlane a_rigid_plane_facing(b3::Vec3 normal) {
    return {
      .plane = { .normal = normal, .offset = 0.0 },
      .push_limit = float::max,
      .push = 0.0,
      .clip_velocity = true,
    };
}

fn PlaneSolve the_solve_of(b3::PlaneSolverResult result, b3::CollisionPlane[] planes) {
    return {
      .delta = result.delta,
      .iteration_count = result.iteration_count,
      .converged_on_the_first_pass = result.iteration_count == 0,
      .plane_count = (int)planes.len,
      .first_push = planes.len > 0 ? planes[0].push : 0.0f,
      .second_push = planes.len > 1 ? planes[1].push : 0.0f,
    };
}

struct GroundSolves {
    PlaneSolve driven_in;
    PlaneSolve already_clear;
    PlaneSolve softly_limited;
    PlaneSolve into_a_corner;
    PlaneSolve against_no_planes;
    PlaneSolve between_two_it_cannot_satisfy;
}

// A metre straight down into the ground plane, straight up away from it, down against a plane whose
// push_limit stops it a quarter of the way, and down and back into the corner of two. The first
// carries a push the solve must discard; the last pair demands two places of the mover at once.
fn GroundSolves the_solves_against_a_ground_plane() {
    b3::Vec3 a_metre_down = { 0.0, -1.0, 0.0 };
    b3::Vec3 up = { 0.0, 1.0, 0.0 };
    b3::Vec3 down = { 0.0, -1.0, 0.0 };

    b3::CollisionPlane[1] driven_in = { a_rigid_plane_facing(up) };
    driven_in[0].push = A_PUSH_NO_SOLVE_PRODUCES;
    b3::PlaneSolverResult drove_in = b3::solve_planes(a_metre_down, driven_in[..]);

    b3::CollisionPlane[1] already_clear = { a_rigid_plane_facing(up) };
    b3::PlaneSolverResult cleared = b3::solve_planes({ 0.0, 1.0, 0.0 }, already_clear[..]);

    b3::CollisionPlane[1] softly_limited = { a_rigid_plane_facing(up) };
    softly_limited[0].push_limit = 0.25;
    b3::PlaneSolverResult limited = b3::solve_planes(a_metre_down, softly_limited[..]);

    b3::CollisionPlane[2] into_a_corner = {
        a_rigid_plane_facing(up),
        a_rigid_plane_facing({ 1.0, 0.0, 0.0 }),
    };
    b3::PlaneSolverResult wedged = b3::solve_planes({ -1.0, -1.0, 0.0 }, into_a_corner[..]);

    b3::CollisionPlane[] no_planes = {};
    b3::PlaneSolverResult unopposed = b3::solve_planes(a_metre_down, no_planes);

    b3::CollisionPlane[2] opposed = { a_rigid_plane_facing(up), a_rigid_plane_facing(down) };
    opposed[1].plane.offset = AN_OPPOSING_OFFSET;
    b3::PlaneSolverResult never_settled = b3::solve_planes(a_metre_down, opposed[..]);

    return {
      .driven_in = the_solve_of(drove_in, driven_in[..]),
      .already_clear = the_solve_of(cleared, already_clear[..]),
      .softly_limited = the_solve_of(limited, softly_limited[..]),
      .into_a_corner = the_solve_of(wedged, into_a_corner[..]),
      .against_no_planes = the_solve_of(unopposed, no_planes),
      .between_two_it_cannot_satisfy = the_solve_of(never_settled, opposed[..]),
    };
}

fn void the_plane_solver_rests_one_linear_slop_inside_every_plane_it_leans_on() @test {
    GroundSolves solves = the_solves_against_a_ground_plane();
    float slop = a_linear_slop();

    assert(solves.driven_in.plane_count == 1, "the solve was handed the one ground plane");
    assert(solves.driven_in.delta.y > -1.5f * slop && solves.driven_in.delta.y < -0.5f * slop,
        "a metre driven straight down comes back resting a linear slop under the plane, not on it");
    assert(math::abs(solves.driven_in.delta.x) < 0.0001 && math::abs(solves.driven_in.delta.z) < 0.0001
        && solves.driven_in.iteration_count == 1,
        "moved along the plane normal alone, and settled on the pass after the one that moved it");

    assert(solves.into_a_corner.plane_count == 2, "the corner is two planes");
    assert(solves.into_a_corner.delta.x > -1.5f * slop && solves.into_a_corner.delta.x < -0.5f * slop
        && solves.into_a_corner.delta.y > -1.5f * slop && solves.into_a_corner.delta.y < -0.5f * slop,
        "and a metre into both of them rests a slop inside each rather than a slop from the corner");
}

fn void the_plane_solver_rewrites_the_push_it_was_handed() @test {
    GroundSolves solves = the_solves_against_a_ground_plane();
    float slop = a_linear_slop();
    float pushed_out = 1.0f - slop;

    assert(solves.driven_in.plane_count == 1, "the solve was handed the one ground plane");
    assert(solves.driven_in.first_push > pushed_out - 0.5f * slop
        && solves.driven_in.first_push < pushed_out + 0.5f * slop,
        "the push written back is the metre less the slop it stops short by, not that atop what it arrived with");

    assert(solves.into_a_corner.plane_count == 2, "the corner is two planes");
    assert(solves.into_a_corner.first_push > pushed_out - 0.5f * slop
        && solves.into_a_corner.first_push < pushed_out + 0.5f * slop
        && solves.into_a_corner.second_push > pushed_out - 0.5f * slop
        && solves.into_a_corner.second_push < pushed_out + 0.5f * slop,
        "and each plane of a corner carries its own share, which here is the same metre apiece");
}

fn void a_delta_no_plane_opposes_is_returned_whole() @test {
    GroundSolves solves = the_solves_against_a_ground_plane();

    assert(0.9999 < solves.already_clear.delta.y && solves.already_clear.delta.y < 1.0001,
        "a metre straight up off the plane is left exactly as it arrived");
    assert(solves.already_clear.converged_on_the_first_pass
        && math::abs(solves.already_clear.first_push) < 0.0001,
        "the first pass converging without moving anything, so the plane is left pushing nothing");

    assert(-1.0001 < solves.against_no_planes.delta.y && solves.against_no_planes.delta.y < -0.9999,
        "and a metre down against no planes at all is answered rather than refused");
    assert(solves.against_no_planes.converged_on_the_first_pass
        && solves.against_no_planes.plane_count == 0,
        "on an empty slice, which converges on that same first pass for the same reason");
}

fn void a_push_limit_stops_the_solve_short_of_the_plane() @test {
    GroundSolves solves = the_solves_against_a_ground_plane();

    assert(solves.softly_limited.plane_count == 1, "the limited solve was handed the one plane");
    assert(-0.7501 < solves.softly_limited.delta.y && solves.softly_limited.delta.y < -0.7499,
        "a limit of a quarter metre leaves three quarters of the metre unrecovered");
    assert(0.2499 < solves.softly_limited.first_push && solves.softly_limited.first_push < 0.2501,
        "the plane having pushed exactly its limit rather than what the separation asked for");
    assert(solves.softly_limited.iteration_count == 1,
        "and stopping there, the clamp leaving nothing more for a later pass to add");
}

fn void a_delta_no_arrangement_of_planes_admits_runs_every_pass_the_solver_has() @test {
    GroundSolves solves = the_solves_against_a_ground_plane();
    float slop = a_linear_slop();

    assert(solves.between_two_it_cannot_satisfy.iteration_count == 20,
        "two planes demanding the mover be above the ground and below it at once never converge");
    assert(solves.between_two_it_cannot_satisfy.delta.y < -2.0f * slop
        && solves.between_two_it_cannot_satisfy.delta.y > -(AN_OPPOSING_OFFSET + slop),
        "leaving a delta that clears neither of them, well inside one and short of the other");
}

struct GroundClips {
    b3::Vec3 into_the_plane;
    b3::Vec3 away_from_the_plane;
    b3::Vec3 across_an_unsolved_plane;
    b3::Vec3 beside_a_wall_the_solve_never_leaned_on;
    float the_push_the_ground_took;
    bool the_wall_took_no_push;
    b3::Vec3 across_a_plane_that_does_not_clip;
    b3::Vec3 across_no_planes;
    float the_push_behind_the_plane_that_will_not_clip;
}

// The same metre down and out along x against a plane a solve has already leaned on, then the same
// vector reversed, then the planes a clip skips: one no solve ever pushed, the wall of a corner a
// downward solve had no reason to push, and one that refuses to clip — then against none at all.
fn GroundClips the_clips_against_a_ground_plane() {
    b3::Vec3 up = { 0.0, 1.0, 0.0 };
    b3::Vec3 down_and_out = { 1.0, -1.0, 0.0 };

    b3::CollisionPlane[1] solved = { a_rigid_plane_facing(up) };
    b3::solve_planes({ 0.0, -1.0, 0.0 }, solved[..]);
    b3::CollisionPlane[1] unsolved = { a_rigid_plane_facing(up) };
    b3::CollisionPlane[2] ground_and_wall = {
        a_rigid_plane_facing(up),
        a_rigid_plane_facing({ 1.0, 0.0, 0.0 }),
    };
    b3::solve_planes({ 0.0, -1.0, 0.0 }, ground_and_wall[..]);
    b3::CollisionPlane[1] not_clipping = solved;
    not_clipping[0].clip_velocity = false;
    b3::CollisionPlane[] no_planes = {};

    return {
      .into_the_plane = b3::clip_vector(down_and_out, solved[..]),
      .away_from_the_plane = b3::clip_vector({ 1.0, 1.0, 0.0 }, solved[..]),
      .across_an_unsolved_plane = b3::clip_vector(down_and_out, unsolved[..]),
      .beside_a_wall_the_solve_never_leaned_on = b3::clip_vector({ -1.0, -1.0, 0.0 }, ground_and_wall[..]),
      .the_push_the_ground_took = ground_and_wall[0].push,
      .the_wall_took_no_push = ground_and_wall[1].push == 0.0f,
      .across_a_plane_that_does_not_clip = b3::clip_vector(down_and_out, not_clipping[..]),
      .across_no_planes = b3::clip_vector(down_and_out, no_planes),
      .the_push_behind_the_plane_that_will_not_clip = not_clipping[0].push,
    };
}

fn void a_velocity_clip_removes_the_component_running_into_the_plane_and_keeps_the_rest() @test {
    GroundClips clips = the_clips_against_a_ground_plane();

    assert(0.9999 < clips.into_the_plane.x && clips.into_the_plane.x < 1.0001,
        "the metre along the plane survives the clip whole");
    assert(math::abs(clips.into_the_plane.y) < 0.0001 && math::abs(clips.into_the_plane.z) < 0.0001,
        "while the metre into it is taken off exactly, leaving nothing along the normal");

    assert(0.9999 < clips.away_from_the_plane.x && clips.away_from_the_plane.x < 1.0001
        && 0.9999 < clips.away_from_the_plane.y && clips.away_from_the_plane.y < 1.0001,
        "and a vector leaving the plane is kept whole, the clip taking only what runs into it");
}

fn void a_velocity_clip_passes_over_every_plane_no_solve_leaned_on() @test {
    GroundClips clips = the_clips_against_a_ground_plane();

    assert(0.9999 < clips.across_an_unsolved_plane.x && clips.across_an_unsolved_plane.x < 1.0001
        && -1.0001 < clips.across_an_unsolved_plane.y && clips.across_an_unsolved_plane.y < -0.9999,
        "a plane pushing nothing is skipped, so a slice never solved leaves the vector as it arrived");
    assert(clips.the_push_the_ground_took > 0.5 && clips.the_wall_took_no_push,
        "a solve driven straight down leans on the ground and leaves the wall beside it pushing nothing");
    assert(math::abs(clips.beside_a_wall_the_solve_never_leaned_on.y) < 0.0001
        && -1.0001 < clips.beside_a_wall_the_solve_never_leaned_on.x
        && clips.beside_a_wall_the_solve_never_leaned_on.x < -0.9999,
        "so one clip takes the fall off against the ground and leaves the run straight into that wall");

    assert(clips.the_push_behind_the_plane_that_will_not_clip > 0.5,
        "the last plane carries a real push of its own, so only its flag can be what skips it");
    assert(-1.0001 < clips.across_a_plane_that_does_not_clip.y
        && clips.across_a_plane_that_does_not_clip.y < -0.9999,
        "as it is, a plane whose clip_velocity is off being passed over whatever it pushes");
    assert(-1.0001 < clips.across_no_planes.y && clips.across_no_planes.y < -0.9999,
        "and an empty slice is answered with the vector itself rather than refused");
}

fn void the_plane_solver_and_the_velocity_clip_allocate_nothing() @test {
    int before = b3::byte_count();

    GroundSolves solves = the_solves_against_a_ground_plane();
    GroundClips clips = the_clips_against_a_ground_plane();

    assert(solves.driven_in.iteration_count == 1 && solves.into_a_corner.plane_count == 2,
        "every solve ran");
    assert(0.9999 < clips.into_the_plane.x && clips.into_the_plane.x < 1.0001, "as did every clip");
    assert(b3::byte_count() == before,
        "and neither takes anything from box3d: both work in the caller's own array");
}

// The wall body's near face, at its centre less this half width. Every mover fixture below is
// placed against that face rather than against the body's own position.
const float WALL_HALF_WIDTH = 1.0;

// The two half widths across the face. They differ, and the mover below stands far enough out along
// the deeper of them that swapping the pair puts the mover outside the wall altogether.
const float WALL_HALF_HEIGHT = 1.5;
const float WALL_HALF_DEPTH = 4.0;

const float MOVER_RADIUS = 0.25;

// How far the mover's surface reaches past the wall's face, which is what the reported plane's
// offset must come back as.
const float A_PENETRATION = 0.15;

// The mover's two centres, which are the span the wall's closest point must come back inside.
const float MOVER_LOW_CENTRE = -0.6;
const float MOVER_HIGH_CENTRE = 0.4;

// How far along the wall's deeper half width the mover stands. It is inside that one and outside the
// shallower, so transposing the mover's two across-the-face axes takes it off the wall entirely.
const float A_MOVER_DEPTH_ALONG_Z = 3.0;

const int A_COLLECTOR_CAPACITY = 8;

// What one mover collide handed a collecting callback. plane_count stops at the capacity above, so
// the fixtures that outrun it read widest_call instead — the largest count any single call was
// given, which is what a per-shape ceiling shows up in.
struct MoverPlanes {
    int call_count;
    int plane_count;
    int widest_call;
    b3::BodyPlaneResult[A_COLLECTOR_CAPACITY] planes;
    bool stop_at_the_first_shape;
}

fn bool collect_mover_planes(
  b3::ShapeId shape_id,
  b3::PlaneResult* planes,
  int plane_count,
  void* context,
) {
    MoverPlanes* collected = (MoverPlanes*)context;
    collected.call_count++;
    if (plane_count > collected.widest_call) collected.widest_call = plane_count;

    for (int i = 0; i < plane_count && collected.plane_count < A_COLLECTOR_CAPACITY; i++) {
        collected.planes[collected.plane_count] = { .shape_id = shape_id, .result = planes[i] };
        collected.plane_count++;
    }
    return !collected.stop_at_the_first_shape;
}

fn bool accept_every_shape(b3::ShapeId shape_id, void* context) {
    int* offered = (int*)context;
    *offered = *offered + 1;
    return true;
}

fn bool refuse_every_shape(b3::ShapeId shape_id, void* context) {
    int* offered = (int*)context;
    *offered = *offered + 1;
    return false;
}

// A static box centred two units out along x, so its near face stands at x = 1, and the body sits at
// zero on the other two axes, so a fixture transposing x with either of them puts the wall somewhere
// the mover never reaches.
fn b3::WorldId a_world_holding_one_wall(b3::ShapeId* out_shape) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = { 2.0, 0.0, 0.0 };
    b3::BodyId body = b3::create_body(world, &body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::BoxHull box = b3::make_box_hull(WALL_HALF_WIDTH, WALL_HALF_HEIGHT, WALL_HALF_DEPTH);
    *out_shape = b3::create_hull_shape(body, &shape_def, &box.base)!!;
    return world;
}

// The capsule every wall fixture pushes into the face, standing upright across it.
fn b3::Capsule an_upright_mover() {
    return {
      .center1 = { 0.0, MOVER_LOW_CENTRE, 0.0 },
      .center2 = { 0.0, MOVER_HIGH_CENTRE, 0.0 },
      .radius = MOVER_RADIUS,
    };
}

fn b3::Pos a_mover_origin_penetrating_the_wall() {
    return { WALL_HALF_WIDTH + A_PENETRATION - MOVER_RADIUS, 0.0, A_MOVER_DEPTH_ALONG_Z };
}

fn MoverPlanes the_planes_of_a_mover_pushed_into_a_wall(b3::WorldId world) {
    b3::Capsule mover = an_upright_mover();
    MoverPlanes collected;
    b3::world_collide_mover(world, origin: a_mover_origin_penetrating_the_wall(),
      mover: &mover,
      filter: b3::default_query_filter(),
      fcn: &collect_mover_planes,
      context: &collected,);
    return collected;
}

fn void a_mover_pushed_into_a_wall_reports_a_plane_pointing_out_of_it() @test {
    b3::ShapeId wall_shape;
    b3::WorldId world = a_world_holding_one_wall(&wall_shape);
    defer b3::destroy_world(world);

    MoverPlanes collected = the_planes_of_a_mover_pushed_into_a_wall(world);
    assert(collected.call_count == 1 && collected.plane_count == 1,
        "the one wall the mover reaches answers once, with the one plane a hull can produce");
    assert(collected.planes[0].shape_id.index1 == wall_shape.index1,
        "and names the wall's own shape");

    b3::Plane plane = collected.planes[0].result.plane;
    assert(plane.normal.x > -1.0001f && plane.normal.x < -0.9999f,
        "the normal points back out of the wall, along the axis the mover pushed in on");
    assert(math::abs(plane.normal.y) < 0.0001f && math::abs(plane.normal.z) < 0.0001f,
        "and carries nothing on the two axes across the face");
    assert(plane.offset > A_PENETRATION - 0.0001f && plane.offset < A_PENETRATION + 0.0001f,
        "the offset being how far the mover's surface reached past that face");

    b3::Vec3 point = collected.planes[0].result.point;
    float face_from_the_origin = MOVER_RADIUS - A_PENETRATION;
    assert(point.x > face_from_the_origin - 0.0001f && point.x < face_from_the_origin + 0.0001f,
        "the closest point sits on the face, and is reported relative to the query origin");
    assert(math::abs(point.z) < 0.0001f,
        "square on the axis the mover was offset along, that offset having come off with the origin");
    assert(point.y > MOVER_LOW_CENTRE - 0.0001f && point.y < MOVER_HIGH_CENTRE + 0.0001f,
        "and somewhere along the mover's own segment, which is the only span the face can answer on");
}

fn void the_plane_a_wall_reports_slides_a_mover_along_it() @test {
    b3::ShapeId wall_shape;
    b3::WorldId world = a_world_holding_one_wall(&wall_shape);
    defer b3::destroy_world(world);

    MoverPlanes collected = the_planes_of_a_mover_pushed_into_a_wall(world);
    assert(collected.plane_count == 1, "the wall reported the one plane the chain is fed");

    b3::CollisionPlane[1] planes = {{
      .plane = collected.planes[0].result.plane,
      .push_limit = float::max,
      .push = 0.0,
      .clip_velocity = true,
    }};

    b3::Vec3 into_the_wall_and_along_it = { 1.0, 1.0, 0.0 };
    b3::PlaneSolverResult solved = b3::solve_planes(into_the_wall_and_along_it, planes[..]);
    float slop = a_linear_slop();
    float pushed_out = -(A_PENETRATION - slop);

    assert(solved.delta.x > pushed_out - 0.5f * slop && solved.delta.x < pushed_out + 0.5f * slop,
        "solving that plane backs the mover out by the penetration less one slop");
    assert(solved.delta.y > 0.9999f && solved.delta.y < 1.0001f
        && math::abs(solved.delta.z) < 0.0001f,
        "leaving the metre along the wall untouched");
    assert(solved.iteration_count == 1,
        "and settling on the pass after the one that moved it");

    b3::Vec3 clipped = b3::clip_vector(into_the_wall_and_along_it, planes[..]);
    assert(math::abs(clipped.x) < 0.0001f,
        "clipping that same solved plane takes the whole run into the wall off the velocity");
    assert(clipped.y > 0.9999f && clipped.y < 1.0001f && math::abs(clipped.z) < 0.0001f,
        "and keeps the run along it, which is the slide the two halves together produce");
}

struct MoverCasts {
    float into_the_wall;
    float along_the_wall;
    float with_every_shape_accepted;
    float with_every_shape_refused;
    int shapes_offered_to_the_accepting_filter;
    int shapes_offered_to_the_refusing_filter;
}

// Every cast starts clear of the wall at the world origin's x, so the fraction it reports is the
// whole approach rather than a mover already inside the face.
fn MoverCasts the_casts_of_a_mover_at_a_wall(b3::WorldId world, float translation_length) {
    b3::Capsule mover = an_upright_mover();
    b3::QueryFilter filter = b3::default_query_filter();
    b3::Pos start = { 0.0, 0.0, A_MOVER_DEPTH_ALONG_Z };
    b3::Vec3 at_the_wall = { translation_length, 0.0, 0.0 };
    int accepted = 0;
    int refused = 0;

    float unfiltered = b3::world_cast_mover(world, origin: start,
      mover: &mover,
      translation: at_the_wall,
      filter: filter,
      fcn: null,
      context: null,);
    float aside = b3::world_cast_mover(world, origin: start,
      mover: &mover,
      translation: { 0.0, 0.0, translation_length },
      filter: filter,
      fcn: null,
      context: null,);
    float through_an_accepting_filter = b3::world_cast_mover(world, origin: start,
      mover: &mover,
      translation: at_the_wall,
      filter: filter,
      fcn: &accept_every_shape,
      context: &accepted,);
    float through_a_refusing_filter = b3::world_cast_mover(world, origin: start,
      mover: &mover,
      translation: at_the_wall,
      filter: filter,
      fcn: &refuse_every_shape,
      context: &refused,);

    return {
      .into_the_wall = unfiltered,
      .along_the_wall = aside,
      .with_every_shape_accepted = through_an_accepting_filter,
      .with_every_shape_refused = through_a_refusing_filter,
      .shapes_offered_to_the_accepting_filter = accepted,
      .shapes_offered_to_the_refusing_filter = refused,
    };
}

fn void a_mover_cast_into_a_wall_stops_one_linear_slop_past_its_face() @test {
    b3::ShapeId wall_shape;
    b3::WorldId world = a_world_holding_one_wall(&wall_shape);
    defer b3::destroy_world(world);

    float translation_length = 2.0;
    MoverCasts casts = the_casts_of_a_mover_at_a_wall(world, translation_length);
    float slop = a_linear_slop();
    float surface = casts.into_the_wall * translation_length + MOVER_RADIUS;

    assert(surface > WALL_HALF_WIDTH + 0.5f * slop && surface < WALL_HALF_WIDTH + 1.5f * slop,
        "a quarter-metre mover clears two slops of radius, so the sweep stops a slop inside the face");
    assert(casts.into_the_wall > 0.37f && casts.into_the_wall < 0.39f,
        "which is a shade over three eighths of the two metres it was asked to travel");

    assert(casts.along_the_wall > 0.9999f && casts.along_the_wall <= 1.0f,
        "and the same mover sent along the face rather than into it takes the whole translation");
}

fn void a_mover_cast_takes_the_shape_filter_it_is_handed() @test {
    b3::ShapeId wall_shape;
    b3::WorldId world = a_world_holding_one_wall(&wall_shape);
    defer b3::destroy_world(world);

    MoverCasts casts = the_casts_of_a_mover_at_a_wall(world, 2.0);

    assert(casts.shapes_offered_to_the_accepting_filter == 1
        && casts.shapes_offered_to_the_refusing_filter == 1,
        "each filter was offered the one wall the sweep reaches");
    assert(casts.with_every_shape_accepted > casts.into_the_wall - 0.0001f
        && casts.with_every_shape_accepted < casts.into_the_wall + 0.0001f,
        "accepting it leaves the cast where the same cast under no filter at all landed");
    assert(casts.with_every_shape_refused > 0.9999f && casts.with_every_shape_refused <= 1.0f,
        "and refusing it drops the wall from the sweep, which then runs the whole translation");
}

// Three spheres of unequal radius at unequal offsets, so a prefix of them is never a prefix of
// equals, and the mover swallows all three at once.
fn b3::BodyId a_body_carrying_three_spheres(b3::WorldId world) {
    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = { 0.0, 0.0, 0.0 };
    b3::BodyId body = b3::create_body(world, &body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere across = { .center = { 0.4, 0.0, 0.0 }, .radius = 0.30 };
    b3::Sphere above = { .center = { 0.0, 0.5, 0.0 }, .radius = 0.35 };
    b3::Sphere behind = { .center = { 0.0, 0.0, 0.6 }, .radius = 0.40 };
    b3::create_sphere_shape(body, &shape_def, &across)!!;
    b3::create_sphere_shape(body, &shape_def, &above)!!;
    b3::create_sphere_shape(body, &shape_def, &behind)!!;
    return body;
}

fn b3::Capsule a_mover_swallowing_three_spheres() {
    return {
      .center1 = { 0.0, -0.15, 0.0 },
      .center2 = { 0.0, 0.25, 0.0 },
      .radius = 0.5,
    };
}

fn void a_body_local_mover_collide_answers_every_prefix_of_what_it_touches() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::BodyId body = a_body_carrying_three_spheres(world);
    b3::Capsule mover = a_mover_swallowing_three_spheres();
    b3::QueryFilter filter = b3::default_query_filter();
    b3::WorldTransform body_transform = body.transform();

    b3::BodyPlaneResult[4] buffer;
    int[5] reported;
    for (int capacity = 0; capacity <= 4; capacity++) {
        b3::BodyPlaneResult[] planes = b3::body_collide_mover(body, buffer: buffer[:capacity],
          origin: { 0.0, 0.0, 0.0 },
          mover: &mover,
          filter: filter,
          body_transform: body_transform,);
        reported[capacity] = (int)planes.len;
    }

    assert(reported[3] == 3, "all three spheres the mover swallows are reachable at once");
    assert(reported[0] == 0 && reported[1] == 1 && reported[2] == 2,
        "and a buffer short of that is filled to its length rather than refused or left empty");
    assert(reported[4] == 3,
        "while a buffer past the count stops at the count, which is what makes each prefix real");
}

fn void a_body_local_mover_collide_names_the_shape_each_plane_came_from() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::BodyId body = a_body_carrying_three_spheres(world);
    b3::Capsule mover = a_mover_swallowing_three_spheres();

    b3::ShapeId[4] shapes;
    b3::ShapeId[] on_the_body = body.shapes(shapes[..]);
    assert(on_the_body.len == 3, "the body carries the three spheres the fixture gave it");

    b3::BodyPlaneResult[4] buffer;
    b3::BodyPlaneResult[] planes = b3::body_collide_mover(body, buffer: buffer[..],
      origin: { 0.0, 0.0, 0.0 },
      mover: &mover,
      filter: b3::default_query_filter(),
      body_transform: body.transform(),);
    assert(planes.len == 3, "and answers one plane for each of them");

    bool every_plane_named_a_shape_of_this_body = true;
    for (usz i = 0; i < planes.len; i++) {
        bool found = false;
        foreach (shape : on_the_body) {
            if (shape.index1 == planes[i].shape_id.index1) found = true;
        }
        if (!found) every_plane_named_a_shape_of_this_body = false;
    }
    assert(every_plane_named_a_shape_of_this_body,
        "each naming a shape the body actually holds rather than the body itself");
    assert(planes[0].shape_id.index1 != planes[1].shape_id.index1
        && planes[1].shape_id.index1 != planes[2].shape_id.index1
        && planes[0].shape_id.index1 != planes[2].shape_id.index1,
        "and a different one each time, so the prefix walks the shapes rather than repeating one");
}

struct MoverAcrossTheTrees {
    int shapes_reported;
    int shapes_reported_when_the_first_ends_it;
}

// Two static shapes, one kinematic and one dynamic, all inside the mover. The static pair is what
// makes the ending callback legible: one tree holds two shapes and the other two hold one each.
fn MoverAcrossTheTrees the_mover_collide_across_the_body_types() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.3 };

    b3::BodyDef static_def = b3::default_body_def();
    static_def.type = b3::BodyType.STATIC;
    static_def.position = { 0.45, 0.0, 0.0 };
    b3::create_sphere_shape(b3::create_body(world, &static_def)!!, &shape_def, &sphere)!!;
    static_def.position = { -0.35, 0.0, 0.0 };
    b3::create_sphere_shape(b3::create_body(world, &static_def)!!, &shape_def, &sphere)!!;

    b3::BodyDef kinematic_def = b3::default_body_def();
    kinematic_def.type = b3::BodyType.KINEMATIC;
    kinematic_def.position = { 0.0, 0.0, 0.4 };
    b3::create_sphere_shape(b3::create_body(world, &kinematic_def)!!, &shape_def, &sphere)!!;

    b3::BodyDef dynamic_def = b3::default_body_def();
    dynamic_def.type = b3::BodyType.DYNAMIC;
    dynamic_def.position = { 0.0, 0.0, -0.5 };
    dynamic_def.gravity_scale = 0.0;
    b3::create_sphere_shape(b3::create_body(world, &dynamic_def)!!, &shape_def, &sphere)!!;

    b3::Capsule mover = {
      .center1 = { 0.0, -0.1, 0.0 },
      .center2 = { 0.0, 0.2, 0.0 },
      .radius = 0.35,
    };
    b3::QueryFilter filter = b3::default_query_filter();

    MoverPlanes everything;
    b3::world_collide_mover(world, origin: { 0.0, 0.0, 0.0 },
      mover: &mover,
      filter: filter,
      fcn: &collect_mover_planes,
      context: &everything,);

    MoverPlanes ended_early;
    ended_early.stop_at_the_first_shape = true;
    b3::world_collide_mover(world, origin: { 0.0, 0.0, 0.0 },
      mover: &mover,
      filter: filter,
      fcn: &collect_mover_planes,
      context: &ended_early,);

    return {
      .shapes_reported = everything.call_count,
      .shapes_reported_when_the_first_ends_it = ended_early.call_count,
    };
}

fn void a_mover_collide_reaches_a_static_a_kinematic_and_a_dynamic_shape_alike() @test {
    MoverAcrossTheTrees across = the_mover_collide_across_the_body_types();

    assert(across.shapes_reported == 4,
        "all four shapes report, so no body type is left out of the mover's reach");
    assert(across.shapes_reported_when_the_first_ends_it == 3,
        "and a callback ending the query ends only the tree it was called from, one type of three");
}

// A grid of small triangles, so the count of planes one shape answers with tracks the mover's reach
// until box3d's own per-shape ceiling stops it.
const int A_GRID_SIDE = 40;
const float A_GRID_CELL = 0.1;

struct MeshPlaneCounts {
    int under_a_small_mover;
    int under_a_wider_mover;
    int under_a_broad_mover;
    int under_the_broadest_mover;
}

fn MeshPlaneCounts the_plane_counts_of_one_mesh_shape() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::MeshData*? grid = b3::create_grid_mesh(
      x_count: A_GRID_SIDE,
      z_count: A_GRID_SIDE,
      cell_width: A_GRID_CELL,
      material_count: 1,
      identify_edges: false,
    );
    defer if (try grid) b3::destroy_mesh(grid);
    b3::MeshData* mesh = grid!!;

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = { 0.0, 0.0, 0.0 };
    b3::BodyId body = b3::create_body(world, &body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::create_mesh_shape(body, &shape_def, mesh, { 1.0, 1.0, 1.0 })!!;

    float[4] radii = { 0.06, 0.12, 0.3, 1.0 };
    int[4] counts;
    for (int i = 0; i < 4; i++) {
        b3::Capsule mover = {
          .center1 = { 0.0, 0.02, 0.0 },
          .center2 = { 0.0, 0.09, 0.0 },
          .radius = radii[i],
        };
        MoverPlanes collected;
        b3::world_collide_mover(world, origin: { 0.0, 0.0, 0.0 },
          mover: &mover,
          filter: b3::default_query_filter(),
          fcn: &collect_mover_planes,
          context: &collected,);
        counts[i] = collected.call_count == 1 ? collected.widest_call : -1;
    }

    return {
      .under_a_small_mover = counts[0],
      .under_a_wider_mover = counts[1],
      .under_a_broad_mover = counts[2],
      .under_the_broadest_mover = counts[3],
    };
}

fn void one_shape_answers_a_mover_with_at_most_sixty_four_planes() @test {
    MeshPlaneCounts counts = the_plane_counts_of_one_mesh_shape();
    int a_shapes_plane_ceiling = 64;

    assert(counts.under_a_small_mover > 0 && counts.under_a_small_mover < counts.under_a_wider_mover,
        "a wider mover reaches more of the grid's triangles, so the count is the reach it found");
    assert(counts.under_a_wider_mover < a_shapes_plane_ceiling,
        "and below the ceiling that count is the whole answer");
    assert(counts.under_a_broad_mover == a_shapes_plane_ceiling
        && counts.under_the_broadest_mover == a_shapes_plane_ceiling,
        "past it two movers of very different reach answer alike, the rest cut without a signal");
}

struct MoverQueryReadings {
    int planes_from_the_world;
    int planes_from_the_body;
    float cast_into_the_wall;
    float cast_along_the_wall;
}

// The world lives and dies inside this call, so the byte count its caller reads afterwards is
// measuring the three queries rather than a world still standing.
fn MoverQueryReadings the_readings_of_every_mover_query() {
    b3::ShapeId wall_shape;
    b3::WorldId world = a_world_holding_one_wall(&wall_shape);
    defer b3::destroy_world(world);

    MoverPlanes collected = the_planes_of_a_mover_pushed_into_a_wall(world);
    MoverCasts casts = the_casts_of_a_mover_at_a_wall(world, 2.0);

    b3::BodyId body = a_body_carrying_three_spheres(world);
    b3::Capsule mover = a_mover_swallowing_three_spheres();
    b3::BodyPlaneResult[4] buffer;
    b3::BodyPlaneResult[] planes = b3::body_collide_mover(body, buffer: buffer[..],
      origin: { 0.0, 0.0, 0.0 },
      mover: &mover,
      filter: b3::default_query_filter(),
      body_transform: body.transform(),);

    return {
      .planes_from_the_world = collected.plane_count,
      .planes_from_the_body = (int)planes.len,
      .cast_into_the_wall = casts.into_the_wall,
      .cast_along_the_wall = casts.along_the_wall,
    };
}

fn void the_mover_queries_allocate_nothing() @test {
    int before = b3::byte_count();
    MoverQueryReadings readings = the_readings_of_every_mover_query();

    assert(readings.planes_from_the_world == 1 && readings.planes_from_the_body == 3,
        "both collides ran and answered");
    assert(readings.cast_into_the_wall < 0.5f && readings.cast_along_the_wall > 0.9999f,
        "as did both casts");
    assert(b3::byte_count() == before,
        "and none of the three takes anything from box3d: each works in the caller's own memory");
}

struct MoverAgainstAMeshOnABody {
    int planes_from_the_body;
    bool the_body_named_the_sphere;
    int shapes_the_world_reported;
}

// One body carrying a mesh and a sphere the mover swallows together, so the two entry points differ
// on the same body under the same mover rather than on two arrangements that could differ anyway.
fn MoverAgainstAMeshOnABody the_readings_of_a_body_carrying_a_mesh_and_a_sphere() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::MeshData*? grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 0.2,
      material_count: 1,
      identify_edges: false,
    );
    defer if (try grid) b3::destroy_mesh(grid);
    b3::MeshData* mesh = grid!!;

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = { 0.0, 0.0, 0.0 };
    b3::BodyId body = b3::create_body(world, &body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::create_mesh_shape(body, &shape_def, mesh, { 1.0, 1.0, 1.0 })!!;
    b3::Sphere sphere = { .center = { 0.0, 0.15, 0.0 }, .radius = 0.2 };
    b3::ShapeId sphere_shape = b3::create_sphere_shape(body, &shape_def, &sphere)!!;

    b3::Capsule mover = {
      .center1 = { 0.0, 0.05, 0.0 },
      .center2 = { 0.0, 0.25, 0.0 },
      .radius = 0.3,
    };
    b3::QueryFilter filter = b3::default_query_filter();

    b3::BodyPlaneResult[8] buffer;
    b3::BodyPlaneResult[] planes = b3::body_collide_mover(body, buffer: buffer[..],
      origin: { 0.0, 0.0, 0.0 },
      mover: &mover,
      filter: filter,
      body_transform: body.transform(),);

    MoverPlanes collected;
    b3::world_collide_mover(world, origin: { 0.0, 0.0, 0.0 },
      mover: &mover,
      filter: filter,
      fcn: &collect_mover_planes,
      context: &collected,);

    return {
      .planes_from_the_body = (int)planes.len,
      .the_body_named_the_sphere = planes.len == 1
        && planes[0].shape_id.index1 == sphere_shape.index1,
      .shapes_the_world_reported = collected.call_count,
    };
}

fn void a_body_local_mover_collide_passes_over_a_shape_the_world_form_reports() @test {
    MoverAgainstAMeshOnABody readings = the_readings_of_a_body_carrying_a_mesh_and_a_sphere();

    assert(readings.shapes_the_world_reported == 2,
        "the mover swallows both shapes on the body, and the world form answers for each");
    assert(readings.planes_from_the_body == 1,
        "the body form answers for one of them, the mesh being none of the three kinds it walks");
    assert(readings.the_body_named_the_sphere,
        "and the one it answers for is the sphere rather than the mesh it stepped over");
}

// The sphere every explosion fixture below throws, and how far from the blast its body stands. Both
// numbers reach the reading: the impulse scales with the area the sphere turns to the blast and
// against the mass its radius gives it, and the distance places it on the falloff ramp.
const float A_RING_SPHERE_RADIUS = 0.5;
const float A_RING_DISTANCE = 2.0;

// The blast every ring fixture is measured under. A sphere's near surface stands half a metre past
// the blast's own radius, so the ring sits on the falloff ramp rather than at full strength, and a
// falloff that never reached box3d would cull the whole ring instead of weakening it.
const float A_BLAST_RADIUS = 1.0;
const float A_BLAST_FALLOFF = 3.0;
const float AN_IMPULSE_PER_AREA = 10.0;

// What one sphere of the ring comes away with under that blast, outward under a positive impulse
// per area and inward under a negative one, and the band either reading is taken inside.
const float A_RING_SPEED = 0.0125;
const float A_SPEED_BAND = 0.0002;

// A speed two orders below the ring's own, so a body reading under it took nothing from the blast.
const float AN_UNTOUCHED_SPEED = 0.0001;

// What the sphere standing around the blast point itself comes away with: the full strength of the
// impulse, no falloff reaching a body the blast is inside, and along an axis box3d falls back on
// because a blast at a body's own centre gives it no direction to be thrown in.
const float A_SPEED_AT_THE_BLAST_POINT = 0.015;

// The four dynamic spheres standing around a blast and the static one standing with them, under one
// explosion. A still body's zero velocity is this struct's own zero too, so the static reading is
// carried as a bool, beside a position no zeroed struct holds and which proves the body was read.
struct RingVelocities {
    bool the_blast_was_accepted;
    b3::Vec3 east;
    b3::Vec3 west;
    b3::Vec3 north;
    b3::Vec3 south;
    b3::Pos static_position;
    bool the_static_body_stayed_still;
}

// What one body took from the blast at its own centre, and what the same body took from a definition
// with a radius and an impulse in it.
struct DefaultBlastReadings {
    bool the_default_was_accepted;
    bool nothing_moved_at_the_blast_point;
    bool nothing_moved_out_in_the_ring;
    b3::Vec3 at_the_blast_point_when_filled_in;
    b3::Vec3 out_in_the_ring_when_filled_in;
}

struct BlastGuards {
    fault negative_radius;
    fault infinite_radius;
    fault radius_not_a_number;
    fault negative_falloff;
    fault infinite_falloff;
    fault falloff_not_a_number;
    fault infinite_impulse;
    fault impulse_not_a_number;
    fault infinite_position;
    fault position_not_a_number;
    bool negative_impulse_passed;
    bool the_default_definition_passed;
}

fn float the_speed_of(b3::Vec3 velocity) {
    return math::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
}

fn bool stands_still(b3::Vec3 velocity) {
    return math::abs(velocity.x) < AN_UNTOUCHED_SPEED
        && math::abs(velocity.y) < AN_UNTOUCHED_SPEED
        && math::abs(velocity.z) < AN_UNTOUCHED_SPEED;
}

// True when the velocity is the ring's own speed along the direction given and nothing across it,
// which is the one reading a blast at the origin may give a sphere standing on that axis. The ring
// is symmetric, so each of the four is named its own direction rather than a common magnitude.
fn bool thrown_along(b3::Vec3 velocity, b3::Vec3 direction) {
    return math::abs(velocity.x - direction.x * A_RING_SPEED) < A_SPEED_BAND
        && math::abs(velocity.y - direction.y * A_RING_SPEED) < A_SPEED_BAND
        && math::abs(velocity.z - direction.z * A_RING_SPEED) < A_SPEED_BAND;
}

fn b3::WorldId a_world_without_gravity() {
    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = { 0.0, 0.0, 0.0 };
    return b3::create_world(&world_def)!!;
}

fn b3::BodyId a_sphere_standing_at(b3::WorldId world, b3::Pos position, b3::BodyType type) {
    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = type;
    body_def.position = position;
    b3::BodyId body = b3::create_body(world, &body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = A_RING_SPHERE_RADIUS };
    b3::create_sphere_shape(body, &shape_def, &sphere)!!;
    return body;
}

fn b3::ExplosionDef a_blast_of(float impulse_per_area) {
    b3::ExplosionDef def = b3::default_explosion_def();
    def.position = { 0.0, 0.0, 0.0 };
    def.radius = A_BLAST_RADIUS;
    def.falloff = A_BLAST_FALLOFF;
    def.impulse_per_area = impulse_per_area;
    return def;
}

// Four dynamic spheres around the blast and a static one the same distance out on the third axis.
// That fifth body is the whole of the difference between the tree this query walks and the three the
// mover queries walk, and no arrangement of dynamic bodies alone can show it.
fn RingVelocities the_ring_under(float impulse_per_area) {
    b3::WorldId world = a_world_without_gravity();
    defer b3::destroy_world(world);

    b3::BodyId east = a_sphere_standing_at(world, { A_RING_DISTANCE, 0.0, 0.0 }, b3::BodyType.DYNAMIC);
    b3::BodyId west = a_sphere_standing_at(world, { -A_RING_DISTANCE, 0.0, 0.0 }, b3::BodyType.DYNAMIC);
    b3::BodyId north = a_sphere_standing_at(world, { 0.0, A_RING_DISTANCE, 0.0 }, b3::BodyType.DYNAMIC);
    b3::BodyId south = a_sphere_standing_at(world, { 0.0, -A_RING_DISTANCE, 0.0 }, b3::BodyType.DYNAMIC);
    b3::BodyId standing = a_sphere_standing_at(world, { 0.0, 0.0, A_RING_DISTANCE }, b3::BodyType.STATIC);

    RingVelocities ring;
    if (catch b3::world_explode(world, a_blast_of(impulse_per_area))) return ring;

    return {
      .the_blast_was_accepted = true,
      .east = east.linear_velocity(),
      .west = west.linear_velocity(),
      .north = north.linear_velocity(),
      .south = south.linear_velocity(),
      .static_position = standing.position(),
      .the_static_body_stayed_still = stands_still(standing.linear_velocity()),
    };
}

fn void an_explosion_throws_the_dynamic_ring_outward_and_leaves_the_static_body_standing() @test {
    RingVelocities ring = the_ring_under(AN_IMPULSE_PER_AREA);
    assert(ring.the_blast_was_accepted, "the blast the ring is read under was accepted");

    assert(thrown_along(ring.east, { 1.0, 0.0, 0.0 }),
        "the sphere standing out along x is thrown further out along x, and nowhere across it");
    assert(thrown_along(ring.west, { -1.0, 0.0, 0.0 }),
        "and the one behind the blast on that axis away from it the other way");
    assert(thrown_along(ring.north, { 0.0, 1.0, 0.0 }),
        "the sphere out along y likewise along y");
    assert(thrown_along(ring.south, { 0.0, -1.0, 0.0 }),
        "and its opposite the other way, each of the four named its own direction");

    assert(math::abs(ring.static_position.z - A_RING_DISTANCE) < 0.0001f
        && math::abs(ring.static_position.x) < 0.0001f
        && math::abs(ring.static_position.y) < 0.0001f,
        "the static sphere stands as far from the blast as the four it was read beside");
    assert(ring.the_static_body_stayed_still,
        "and takes nothing from it: the explosion queries the dynamic tree and neither of the others");
}

fn void a_negative_impulse_per_area_pulls_the_ring_inward_instead() @test {
    RingVelocities ring = the_ring_under(-AN_IMPULSE_PER_AREA);
    assert(ring.the_blast_was_accepted,
        "a negative impulse per area is the implosion box3d documents, not a definition to refuse");

    assert(thrown_along(ring.east, { -1.0, 0.0, 0.0 }),
        "the sphere standing out along x is pulled back towards the blast");
    assert(thrown_along(ring.west, { 1.0, 0.0, 0.0 }),
        "and the one behind it on that axis forward towards the same place");
    assert(thrown_along(ring.north, { 0.0, -1.0, 0.0 }),
        "the sphere out along y likewise inward");
    assert(thrown_along(ring.south, { 0.0, 1.0, 0.0 }),
        "and its opposite inward the other way, the whole ring closing rather than opening");
}

// A sphere around the blast point itself and one out in the ring. The default definition's radius
// reaches nothing, so the near sphere is the only body it could move at all, and its zero impulse is
// then the only thing left holding that sphere still.
fn DefaultBlastReadings the_readings_of_a_default_explosion() {
    b3::WorldId world = a_world_without_gravity();
    defer b3::destroy_world(world);

    b3::BodyId near = a_sphere_standing_at(world, { 0.0, 0.0, 0.0 }, b3::BodyType.DYNAMIC);
    b3::BodyId far = a_sphere_standing_at(world, { A_RING_DISTANCE, 0.0, 0.0 }, b3::BodyType.DYNAMIC);

    DefaultBlastReadings readings;
    if (catch b3::world_explode(world, b3::default_explosion_def())) return readings;

    readings.the_default_was_accepted = true;
    readings.nothing_moved_at_the_blast_point = stands_still(near.linear_velocity());
    readings.nothing_moved_out_in_the_ring = stands_still(far.linear_velocity());

    near.set_linear_velocity({ 0.0, 0.0, 0.0 });
    far.set_linear_velocity({ 0.0, 0.0, 0.0 });
    b3::world_explode(world, a_blast_of(AN_IMPULSE_PER_AREA))!!;

    readings.at_the_blast_point_when_filled_in = near.linear_velocity();
    readings.out_in_the_ring_when_filled_in = far.linear_velocity();
    return readings;
}

fn void the_default_explosion_definition_moves_nothing() @test {
    DefaultBlastReadings readings = the_readings_of_a_default_explosion();
    assert(readings.the_default_was_accepted,
        "the default definition is a definition box3d takes, not one this binding refuses");

    assert(readings.nothing_moved_at_the_blast_point,
        "the sphere around the blast point takes nothing from a default explosion");
    assert(readings.nothing_moved_out_in_the_ring,
        "and neither does the one out in the ring: radius, falloff and impulse all come back zero");

    assert(thrown_along(readings.out_in_the_ring_when_filled_in, { 1.0, 0.0, 0.0 }),
        "filling that same definition in throws the far sphere out along its own axis");
    float near_speed = the_speed_of(readings.at_the_blast_point_when_filled_in);
    assert(near_speed > A_SPEED_AT_THE_BLAST_POINT - A_SPEED_BAND
        && near_speed < A_SPEED_AT_THE_BLAST_POINT + A_SPEED_BAND,
        "and throws the near one at full strength, in a direction of box3d's own choosing");
}

// One world, one dynamic sphere in reach, and every definition box3d does no more than assert
// against. The last two are the controls: an implosion and the default, neither of them asserted.
fn BlastGuards the_definitions_an_explosion_is_refused_for() {
    b3::WorldId world = a_world_without_gravity();
    defer b3::destroy_world(world);
    a_sphere_standing_at(world, { A_RING_DISTANCE, 0.0, 0.0 }, b3::BodyType.DYNAMIC);

    float infinity = (float)1.0e40;
    float nan = infinity - infinity;

    BlastGuards guards;
    b3::ExplosionDef def = a_blast_of(AN_IMPULSE_PER_AREA);

    def.radius = -A_BLAST_RADIUS;
    if (catch err = b3::world_explode(world, def)) guards.negative_radius = err;
    def.radius = infinity;
    if (catch err = b3::world_explode(world, def)) guards.infinite_radius = err;
    def.radius = nan;
    if (catch err = b3::world_explode(world, def)) guards.radius_not_a_number = err;

    def = a_blast_of(AN_IMPULSE_PER_AREA);
    def.falloff = -A_BLAST_FALLOFF;
    if (catch err = b3::world_explode(world, def)) guards.negative_falloff = err;
    def.falloff = infinity;
    if (catch err = b3::world_explode(world, def)) guards.infinite_falloff = err;
    def.falloff = nan;
    if (catch err = b3::world_explode(world, def)) guards.falloff_not_a_number = err;

    def = a_blast_of(infinity);
    if (catch err = b3::world_explode(world, def)) guards.infinite_impulse = err;
    def = a_blast_of(nan);
    if (catch err = b3::world_explode(world, def)) guards.impulse_not_a_number = err;

    def = a_blast_of(AN_IMPULSE_PER_AREA);
    def.position = { 0.0, 0.0, infinity };
    if (catch err = b3::world_explode(world, def)) guards.infinite_position = err;
    def.position = { 0.0, nan, 0.0 };
    if (catch err = b3::world_explode(world, def)) guards.position_not_a_number = err;

    if (try b3::world_explode(world, a_blast_of(-AN_IMPULSE_PER_AREA))) guards.negative_impulse_passed = true;
    if (try b3::world_explode(world, b3::default_explosion_def())) guards.the_default_definition_passed = true;

    return guards;
}

// box3d asserts a finite position, a finite non-negative radius and falloff, and a finite impulse
// per area, and this build compiles all four assertions out. None of what gets past them is inert,
// and each reading below is what the unguarded call did to a sphere two metres from the blast.
fn void an_explosion_definition_box3d_would_assert_against_is_refused() @test {
    BlastGuards guards = the_definitions_an_explosion_is_refused_for();

    assert(guards.negative_radius == b3::INVALID_DEFINITION,
        "a negative radius explodes smaller rather than not at all");
    assert(guards.infinite_radius == b3::INVALID_DEFINITION,
        "an infinite one reaches every dynamic body in the world at full strength");
    assert(guards.radius_not_a_number == b3::INVALID_DEFINITION,
        "and a NaN radius explodes larger, the falloff ramp comparing false and standing aside");
    assert(guards.negative_falloff == b3::INVALID_DEFINITION,
        "a negative falloff culls every shape and does nothing at all");
    assert(guards.infinite_falloff == b3::INVALID_DEFINITION,
        "an infinite one divides its own ramp by itself and hands back a NaN velocity");
    assert(guards.falloff_not_a_number == b3::INVALID_DEFINITION,
        "and a NaN falloff explodes larger, as a NaN radius does");
    assert(guards.infinite_impulse == b3::INVALID_DEFINITION,
        "an infinite impulse per area sets an infinite velocity");
    assert(guards.impulse_not_a_number == b3::INVALID_DEFINITION,
        "and a NaN one poisons the velocity for the rest of the run, nothing later clearing it");
    assert(guards.infinite_position == b3::INVALID_DEFINITION,
        "a blast at an infinite position is beyond everything and does nothing");
    assert(guards.position_not_a_number == b3::INVALID_DEFINITION,
        "and a NaN position throws every dynamic body in the world at full strength, aimed at nothing");

    assert(guards.negative_impulse_passed,
        "a negative impulse per area is the implosion box3d documents and stays accepted");
    assert(guards.the_default_definition_passed,
        "and the default definition, whose zeroes are all valid, is accepted and moves nothing");
}

fn void an_explosion_allocates_nothing() @test {
    int worlds_before = b3::world_count();
    int bytes_before = b3::byte_count();

    RingVelocities ring = the_ring_under(AN_IMPULSE_PER_AREA);
    assert(ring.the_blast_was_accepted && thrown_along(ring.east, { 1.0, 0.0, 0.0 }),
        "the explosion the byte count is read around ran and threw the ring");
    assert(b3::world_count() == worlds_before && b3::byte_count() == bytes_before,
        "and took nothing from box3d it did not give back: the impulse goes straight onto the bodies");
}
