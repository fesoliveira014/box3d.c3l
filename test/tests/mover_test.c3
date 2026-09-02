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
