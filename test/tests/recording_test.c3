module b3_test;

import b3;
import std::io::path;

const float A_TIME_STEP = 1.0f / 60.0f;
const int A_SUB_STEP_COUNT = 4;
const int A_RECORDED_FRAME_COUNT = 120;

struct DroppedPair {
    b3::WorldId world;
    b3::BodyId west;
    b3::BodyId east;
}

// Two spheres a metre either side of the origin, dropped onto a slab, so a fixture that swapped
// them could not pass. No helper here asserts: C3 skips defer on a panicking assertion, and a
// stranded world or recording sits on world_count() and byte_count() for the rest of the run.
fn DroppedPair a_pair_dropped_on_a_slab() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::BoxHull slab = b3::make_box_hull(5.0, 0.5, 5.0);
    b3::BodyDef ground_def = b3::default_body_def();
    ground_def.position = { 0.0, -0.5, 0.0 };
    b3::BodyId ground = world.create_body(&ground_def)!!;
    ground.create_hull_shape(&shape_def, &slab.base)!!;

    b3::Sphere ball = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::BodyDef ball_def = b3::default_body_def();
    ball_def.type = b3::BodyType.DYNAMIC;
    ball_def.position = { -1.0, 2.0, 0.0 };
    b3::BodyId west = world.create_body(&ball_def)!!;
    west.create_sphere_shape(&shape_def, &ball)!!;
    ball_def.position = { 1.0, 2.0, 0.0 };
    b3::BodyId east = world.create_body(&ball_def)!!;
    east.create_sphere_shape(&shape_def, &ball)!!;

    return { .world = world, .west = west, .east = east };
}

// box3d takes a path rather than a handle, so a saved recording needs somewhere on disk that is
// not the repository. Every test that writes one deletes it before returning.
fn String a_temporary_recording_path(String name) {
    Path directory = path::temp_directory(tmem) ?? (Path)"/tmp";
    return string::tformat("%s/%s", (String)directory, name);
}

struct FreshRecording {
    int baseline;
    int while_alive;
    bool empty_over_live_storage;
}

fn FreshRecording the_readings_around_a_fresh_recording() {
    int baseline = b3::byte_count();

    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    char[] bytes = recording.data();
    return {
        .baseline = baseline,
        .while_alive = b3::byte_count(),
        .empty_over_live_storage = bytes.len == 0 && bytes.ptr != null,
    };
}

fn void a_fresh_recording_is_empty_over_storage_box3d_has_already_taken() @test {
    FreshRecording around = the_readings_around_a_fresh_recording();
    int taken = around.while_alive - around.baseline;

    assert(taken >= 65536 && taken < 66560, "the default capacity is 64 KiB and box3d takes it at creation");
    assert(around.empty_over_live_storage,
        "the buffer is live and holds nothing, where box3d's header calls the pointer null until the first write");
    assert(around.while_alive > around.baseline && b3::byte_count() == around.baseline,
        "and destroying the recording gives every byte back");
}

fn int[4] the_bytes_a_recording_takes_at_each_starting_capacity() {
    int baseline = b3::byte_count();

    b3::Recording* small = b3::create_recording(128);
    int at_128 = b3::byte_count();
    small.destroy();

    b3::Recording* standard = b3::create_recording(0);
    int at_zero = b3::byte_count();
    standard.destroy();

    b3::Recording* backwards = b3::create_recording(-1);
    int at_negative = b3::byte_count();
    backwards.destroy();

    return { baseline, at_128, at_zero, at_negative };
}

fn void a_starting_capacity_below_one_is_read_as_the_default() @test {
    int[4] bytes = the_bytes_a_recording_takes_at_each_starting_capacity();
    int at_128 = bytes[1] - bytes[0];
    int at_zero = bytes[2] - bytes[0];
    int at_negative = bytes[3] - bytes[0];

    assert(at_128 >= 128 && at_128 < 1024, "a starting capacity of 128 takes that buffer and the handle, and no 64 KiB");
    assert(at_zero >= 65536 && at_zero < 66560, "where a capacity of zero takes the 64 KiB default");
    assert(at_negative == at_zero && at_negative >= 65536,
        "and a negative capacity is read as no capacity, not as a negative allocation");
    assert(b3::byte_count() == bytes[0], "all three gave their bytes back");
}

struct RecordedSession {
    int baseline;
    int worlds_before;
    int at_start;
    int at_stop;
    b3::Pos west;
    b3::Pos east;
    bool replays;
}

fn RecordedSession the_readings_from_a_recorded_drop() {
    int baseline = b3::byte_count();
    int worlds_before = b3::world_count();

    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    if (catch pair.world.start_recording(recording)) return { .baseline = baseline, .worlds_before = worlds_before };
    int at_start = recording.data().len;
    for (int frame = 0; frame < A_RECORDED_FRAME_COUNT; frame++) {
        pair.world.step(A_TIME_STEP, A_SUB_STEP_COUNT);
    }
    pair.world.stop_recording();

    return {
        .baseline = baseline,
        .worlds_before = worlds_before,
        .at_start = at_start,
        .at_stop = recording.data().len,
        .west = pair.west.position(),
        .east = pair.east.position(),
        .replays = b3::validate_replay(recording.data(), 1),
    };
}

fn void a_recorded_world_reports_the_bytes_its_session_wrote() @test {
    RecordedSession session = the_readings_from_a_recorded_drop();

    assert(session.at_start > 1000 && session.at_start < 20000,
        "starting a session writes the header and a seed snapshot of the whole world before any step");
    assert(session.at_stop > session.at_start && session.at_stop < 20000,
        "and every recorded step adds to that");
    assert(session.west.x > -1.05 && session.west.x < -0.95 && session.west.y > 0.45 && session.west.y < 0.55,
        "the west sphere came to rest on the slab where it was dropped");
    assert(session.east.x > 0.95 && session.east.x < 1.05 && session.east.y > 0.45 && session.east.y < 0.55,
        "and the east sphere on its own side");
    assert(session.replays, "the recorded bytes replay to every hash they carry");
    assert(session.at_stop > session.at_start && b3::world_count() == session.worlds_before
        && b3::byte_count() == session.baseline,
        "and the recording and the world it came from gave back everything they took");
}

struct FileRoundTrip {
    int written_length;
    int read_length;
    uint written_hash;
    uint read_hash;
    bool read_back_replays;
}

fn FileRoundTrip the_readings_from_a_recording_through_a_file() {
    String file = a_temporary_recording_path("box3d_round_trip.rec");
    defer (void)path::delete((PathCommon)file);

    b3::Recording* written = b3::create_recording(0);
    defer written.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    if (catch pair.world.start_recording(written)) return {};
    for (int frame = 0; frame < A_RECORDED_FRAME_COUNT; frame++) {
        pair.world.step(A_TIME_STEP, A_SUB_STEP_COUNT);
    }
    pair.world.stop_recording();

    if (catch written.save(file.zstr_tcopy())) return {};
    b3::Recording*? read = b3::load_recording(file.zstr_tcopy());
    if (catch read) return { .written_length = written.data().len, .written_hash = b3::hash(written.data()) };
    defer read.destroy();

    return {
        .written_length = written.data().len,
        .read_length = read.data().len,
        .written_hash = b3::hash(written.data()),
        .read_hash = b3::hash(read.data()),
        .read_back_replays = b3::validate_replay(read.data(), 1),
    };
}

fn void a_saved_recording_loads_back_byte_for_byte() @test {
    FileRoundTrip trip = the_readings_from_a_recording_through_a_file();

    assert(trip.written_length > 1000 && trip.written_length < 20000, "the session wrote a recording worth saving");
    assert(trip.read_length == trip.written_length && trip.read_length > 1000,
        "and the file gave back exactly as many bytes as went into it");
    assert(trip.read_hash == trip.written_hash && trip.read_length > 1000,
        "the same bytes, not merely the same count");
    assert(trip.read_back_replays, "and the loaded copy replays as the original did");
}

struct EmptyThroughAFile {
    bool saved;
    fault load_refusal;
}

fn EmptyThroughAFile the_readings_from_an_unrecorded_recording_through_a_file() {
    String file = a_temporary_recording_path("box3d_empty.rec");
    defer (void)path::delete((PathCommon)file);

    b3::Recording* empty = b3::create_recording(0);
    defer empty.destroy();

    EmptyThroughAFile readings;
    readings.saved = @ok(empty.save(file.zstr_tcopy()));
    b3::Recording*? read = b3::load_recording(file.zstr_tcopy());
    if (catch err = read) return { .saved = readings.saved, .load_refusal = err };
    read.destroy();
    return readings;
}

fn void an_unrecorded_recording_saves_and_will_not_load_back() @test {
    EmptyThroughAFile trip = the_readings_from_an_unrecorded_recording_through_a_file();

    assert(trip.saved, "box3d writes an empty file for a recording no session filled, rather than refusing");
    assert(trip.load_refusal == b3::FILE_UNAVAILABLE,
        "and refuses to read that file back, being shorter than the header a recording opens with");
}

struct SilentRefusals {
    int baseline;
    int worlds_before;
    fault save_to_a_missing_directory;
    fault load_a_missing_path;
    bool junk_was_not_replayed;
    fault player_over_junk;
    bool nothing_was_replayed;
    int after;
    int worlds_after;
}

// box3d prints its reason for refusing a player on standard output, so "bad magic" and "recording
// too small" appear in the run's output from here and are not a failure.
fn SilentRefusals the_readings_from_the_four_silent_refusals() {
    char[64] junk;
    for (int i = 0; i < junk.len; i++) junk[i] = 0xAB;

    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    SilentRefusals refusals;
    refusals.baseline = b3::byte_count();
    refusals.worlds_before = b3::world_count();

    if (catch err = recording.save("/no/such/directory/box3d.rec")) refusals.save_to_a_missing_directory = err;
    if (catch err = b3::load_recording("/no/such/directory/box3d.rec")) refusals.load_a_missing_path = err;
    refusals.junk_was_not_replayed = !b3::validate_replay(junk[..], 1);
    if (catch err = b3::create_player(junk[..], 1)) refusals.player_over_junk = err;
    refusals.nothing_was_replayed = !b3::validate_replay(recording.data(), 1);

    refusals.after = b3::byte_count();
    refusals.worlds_after = b3::world_count();
    return refusals;
}

fn void the_four_silent_refusals_arrive_as_faults() @test {
    SilentRefusals refusals = the_readings_from_the_four_silent_refusals();

    assert(refusals.save_to_a_missing_directory == b3::FILE_UNAVAILABLE, "a path box3d cannot open is a fault, not a false");
    assert(refusals.load_a_missing_path == b3::FILE_UNAVAILABLE, "as is a path with nothing behind it");
    assert(refusals.junk_was_not_replayed, "sixty-four arbitrary bytes carry no header to replay");
    assert(refusals.nothing_was_replayed, "and neither does a recording no session wrote to");
    assert(refusals.player_over_junk == b3::NOT_A_RECORDING, "and no player is built over them");
    assert(refusals.baseline > 0 && refusals.after == refusals.baseline
        && refusals.worlds_after == refusals.worlds_before,
        "none of the four took a byte or a world it did not give back");
}

struct StartRefusals {
    fault a_null_recording;
    fault while_already_recording;
    bool the_refused_buffer_stayed_empty;
    bool a_restart_after_a_stop_was_taken;
    int the_first_session;
    int the_second_session;
}

fn StartRefusals the_readings_around_a_refused_start() {
    b3::Recording* first = b3::create_recording(0);
    defer first.destroy();
    b3::Recording* second = b3::create_recording(0);
    defer second.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    StartRefusals refusals;
    if (catch pair.world.start_recording(first)) return refusals;
    pair.world.step(A_TIME_STEP, A_SUB_STEP_COUNT);

    if (catch err = pair.world.start_recording(null)) refusals.a_null_recording = err;
    if (catch err = pair.world.start_recording(second)) refusals.while_already_recording = err;
    refusals.the_refused_buffer_stayed_empty = second.data().len == 0;
    pair.world.stop_recording();
    refusals.the_first_session = first.data().len;

    refusals.a_restart_after_a_stop_was_taken = @ok(pair.world.start_recording(second));
    pair.world.step(A_TIME_STEP, A_SUB_STEP_COUNT);
    pair.world.stop_recording();
    refusals.the_second_session = second.data().len;
    return refusals;
}

struct BufferMoves {
    bool the_session_moved_the_buffer;
    bool the_slice_was_empty_before;
    usz after_the_session;
}

// A capacity far under one session's bytes, so box3d must grow the buffer and hand back a different
// pointer. The claim data() makes is that a slice taken before a recorded step does not survive it,
// and only a buffer that actually grows can show that.
fn BufferMoves the_readings_around_a_growing_buffer() {
    b3::Recording* recording = b3::create_recording(128);
    defer recording.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    BufferMoves moved;
    moved.the_slice_was_empty_before = recording.data().len == 0;
    char* before = recording.data().ptr;

    if (catch pair.world.start_recording(recording)) return moved;
    for (int i = 0; i < 30; i++) pair.world.step(A_TIME_STEP, A_SUB_STEP_COUNT);
    pair.world.stop_recording();

    moved.the_session_moved_the_buffer = recording.data().ptr != before;
    moved.after_the_session = recording.data().len;
    return moved;
}

fn void a_recorded_step_moves_the_buffer_out_from_under_an_older_slice() @test {
    BufferMoves moved = the_readings_around_a_growing_buffer();

    assert(moved.the_slice_was_empty_before, "a recording no session has written to hands back no bytes");
    assert(moved.after_the_session > 1000 && moved.after_the_session < 100000,
        "where one that recorded thirty steps hands back the bytes they took");
    assert(moved.the_session_moved_the_buffer,
        "and grew past its own capacity to hold them, so the pointer an earlier slice held is gone");
}

fn void a_world_that_is_already_recording_refuses_a_second_buffer() @test {
    StartRefusals refusals = the_readings_around_a_refused_start();

    assert(refusals.a_null_recording == b3::NOT_A_RECORDING, "box3d takes no recording from a null pointer");
    assert(refusals.while_already_recording == b3::NOT_A_RECORDING, "nor a second buffer while a session is open");
    assert(refusals.the_refused_buffer_stayed_empty, "the buffer it refused is the empty one it never wrote a header into");
    assert(refusals.the_first_session > 1000 && refusals.the_first_session < 100000,
        "the open session went on writing into the buffer it did take");
    assert(refusals.a_restart_after_a_stop_was_taken, "and the same world takes that buffer once the session is stopped");
    assert(refusals.the_second_session > 1000 && refusals.the_second_session < 100000,
        "which then fills, so the refusal was the open session and not anything about that buffer");
}

struct PlayerLifetime {
    int worlds_before;
    int worlds_with_a_player;
    int worlds_after;
    int baseline;
    int after;
    float unit_the_caller_had;
    float unit_before;
    float unit_with_a_player;
    float unit_after;
}

// The player takes the process-wide length unit over, so this reads it against a caller's unit of
// three and puts that unit back before the assertions run rather than in a defer.
fn PlayerLifetime the_readings_around_a_live_player() {
    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    PlayerLifetime around;
    if (catch pair.world.start_recording(recording)) return around;
    pair.world.step(A_TIME_STEP, A_SUB_STEP_COUNT);
    pair.world.stop_recording();

    around.baseline = b3::byte_count();
    around.worlds_before = b3::world_count();
    around.unit_the_caller_had = b3::length_units_per_meter();
    b3::set_length_units_per_meter(3.0);
    around.unit_before = b3::length_units_per_meter();

    b3::RecPlayer*? player = b3::create_player(recording.data(), 1);
    if (catch player) {
        b3::set_length_units_per_meter(1.0);
        return around;
    }
    around.worlds_with_a_player = b3::world_count();
    around.unit_with_a_player = b3::length_units_per_meter();
    player.destroy();

    around.unit_after = b3::length_units_per_meter();
    b3::set_length_units_per_meter(around.unit_the_caller_had);
    around.worlds_after = b3::world_count();
    around.after = b3::byte_count();
    return around;
}

fn void a_player_owns_a_world_and_the_length_unit_while_it_lives() @test {
    PlayerLifetime around = the_readings_around_a_live_player();

    assert(around.worlds_with_a_player == around.worlds_before + 1,
        "the replay world box3d stands up for a player is on the same count a leaked world would shift");
    assert(around.worlds_with_a_player > around.worlds_after && around.worlds_after == around.worlds_before,
        "and destroying the player takes it down again");
    assert(around.baseline > 0 && around.after == around.baseline, "with every byte the player held given back");
    assert(around.unit_before == 3.0 && around.unit_with_a_player == 1.0,
        "a live player replaces the caller's length unit with the one its recording was made under");
    assert(around.unit_after == 3.0, "and destroying it restores the caller's");
}

const int A_DROPPED_BODY_COUNT = 3;
const int A_SEEK_TARGET = 30;

struct ReplayedDrop {
    int recorded_bytes;
    b3::Pos live_west;
    b3::Pos live_east;
    int frames;
    int frame_count;
    int bodies;
    bool every_state_hash_matched;
    fault no_divergence;
    b3::Pos[A_DROPPED_BODY_COUNT] replayed;
    bool every_ordinal_named_a_body;
    int worlds_the_player_added;
    int worlds_left_after_it;
}

fn ReplayedDrop the_readings_from_a_replayed_drop() {
    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    ReplayedDrop replay;
    if (catch pair.world.start_recording(recording)) return replay;
    for (int frame = 0; frame < A_RECORDED_FRAME_COUNT; frame++) {
        pair.world.step(A_TIME_STEP, A_SUB_STEP_COUNT);
    }
    pair.world.stop_recording();

    replay.recorded_bytes = recording.data().len;
    replay.live_west = pair.west.position();
    replay.live_east = pair.east.position();

    int worlds_before = b3::world_count();
    b3::RecPlayer*? player = b3::create_player(recording.data(), 1);
    if (catch player) return replay;
    replay.worlds_the_player_added = b3::world_count() - worlds_before;

    while (player.step_frame()) {}
    replay.frames = player.frame();
    replay.frame_count = player.frame_count();
    replay.bodies = player.body_count();
    replay.every_state_hash_matched = !player.has_diverged();
    if (catch err = player.diverge_frame()) replay.no_divergence = err;

    bool named = replay.bodies == A_DROPPED_BODY_COUNT;
    for (int ordinal = 0; ordinal < A_DROPPED_BODY_COUNT && ordinal < replay.bodies; ordinal++) {
        b3::BodyId? body = player.body_id(ordinal);
        if (catch body) {
            named = false;
            continue;
        }
        replay.replayed[ordinal] = body.position();
    }
    replay.every_ordinal_named_a_body = named;

    player.destroy();
    replay.worlds_left_after_it = b3::world_count() - worlds_before;
    return replay;
}

fn void a_recorded_world_replays_to_the_positions_it_recorded() @test {
    ReplayedDrop replay = the_readings_from_a_replayed_drop();

    assert(replay.recorded_bytes > 1000 && replay.recorded_bytes < 20000,
        "the session wrote a recording worth replaying");
    assert(replay.live_west.x > -1.05 && replay.live_west.x < -0.95
        && replay.live_west.y > 0.45 && replay.live_west.y < 0.55,
        "the recorded world left the west sphere resting on the slab");
    assert(replay.live_east.x > 0.95 && replay.live_east.x < 1.05
        && replay.live_east.y > 0.45 && replay.live_east.y < 0.55,
        "and the east sphere resting on its own side of it");
    assert(replay.frames == A_RECORDED_FRAME_COUNT && replay.frame_count == A_RECORDED_FRAME_COUNT,
        "the replay ran out of records on the last frame the session recorded");
    assert(replay.bodies == A_DROPPED_BODY_COUNT && replay.every_ordinal_named_a_body,
        "and stood up three bodies, the static slab being one of them");
    assert(replay.replayed[1].x == replay.live_west.x && replay.replayed[1].y == replay.live_west.y
        && replay.replayed[1].z == replay.live_west.z,
        "the west sphere came to rest exactly where the recorded world left it, not merely near it");
    assert(replay.replayed[2].x == replay.live_east.x && replay.replayed[2].y == replay.live_east.y
        && replay.replayed[2].z == replay.live_east.z,
        "and so did the east one, which a replay that transposed the pair would fail");
    assert(replay.replayed[0].y > -0.55 && replay.replayed[0].y < -0.45,
        "with the slab back at the height it was built at");
    assert(replay.every_state_hash_matched && replay.no_divergence == b3::NO_HIT,
        "and every state hash the recording carries was reproduced");
    assert(replay.worlds_the_player_added == 1 && replay.worlds_left_after_it == 0,
        "the world the player stood up for the replay went back when it was destroyed");
}

struct SeekReadings {
    b3::Pos stepping_forward;
    b3::Pos after_seeking_back;
    b3::Pos after_a_restart;
    int frame_after_the_seek;
    int frame_after_the_restart;
    int frame_after_overshooting;
    int frame_after_undershooting;
    bool the_end_was_reached_by_overshooting;
    bool the_stream_was_live_at_frame_zero;
    usz keyframe_bytes_after_a_replay;
}

fn SeekReadings the_readings_around_a_seek() {
    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    SeekReadings around;
    if (catch pair.world.start_recording(recording)) return around;
    for (int frame = 0; frame < A_RECORDED_FRAME_COUNT; frame++) {
        pair.world.step(A_TIME_STEP, A_SUB_STEP_COUNT);
    }
    pair.world.stop_recording();

    b3::RecPlayer*? player = b3::create_player(recording.data(), 1);
    if (catch player) return around;

    for (int frame = 0; frame < A_SEEK_TARGET; frame++) player.step_frame();
    if (try west = player.body_id(1)) around.stepping_forward = west.position();

    while (player.step_frame()) {}
    around.keyframe_bytes_after_a_replay = player.keyframe_bytes();

    player.seek_frame(A_SEEK_TARGET);
    around.frame_after_the_seek = player.frame();
    if (try west = player.body_id(1)) around.after_seeking_back = west.position();

    player.seek_frame(2 * A_RECORDED_FRAME_COUNT);
    around.frame_after_overshooting = player.frame();
    around.the_end_was_reached_by_overshooting = player.is_at_end();

    player.seek_frame(-A_SEEK_TARGET);
    around.frame_after_undershooting = player.frame();
    around.the_stream_was_live_at_frame_zero = !player.is_at_end();

    player.restart();
    around.frame_after_the_restart = player.frame();
    if (try west = player.body_id(1)) around.after_a_restart = west.position();

    player.destroy();
    return around;
}

fn void a_seek_puts_the_replay_back_on_a_frame_it_has_already_left() @test {
    SeekReadings around = the_readings_around_a_seek();

    assert(around.stepping_forward.y > 0.70 && around.stepping_forward.y < 0.78,
        "thirty frames in, the west sphere is part way through its fall");
    assert(around.keyframe_bytes_after_a_replay > 0,
        "and a full replay filled the keyframe ring a backward seek restores from");
    assert(around.frame_after_the_seek == A_SEEK_TARGET,
        "seeking back to that frame from the end of the recording lands on it");
    assert(around.after_seeking_back.x == around.stepping_forward.x
        && around.after_seeking_back.y == around.stepping_forward.y
        && around.after_seeking_back.z == around.stepping_forward.z,
        "with the sphere exactly where the forward pass had it, not merely near it");
    assert(around.frame_after_overshooting == A_RECORDED_FRAME_COUNT
        && around.the_end_was_reached_by_overshooting,
        "a target past the recording stops on its last frame rather than being refused");
    assert(around.frame_after_undershooting == 0 && around.the_stream_was_live_at_frame_zero,
        "and a negative target lands on frame 0 with the whole stream still to replay");
    assert(around.frame_after_the_restart == 0
        && around.after_a_restart.y > 1.95 && around.after_a_restart.y < 2.05,
        "a restart puts the sphere back at the height the session was started at");
}

const int A_BUSY_FRAME_COUNT = 60;
const int A_PEBBLE_STEP = 10;
const int A_QUERY_STEP = 30;
const int A_PEBBLE_END_STEP = 40;
const int A_PEBBLE_ORDINAL = 3;
const int A_TAGGED_RAY_ID = 7;
const int A_TAGGED_BOX_ID = 9;

// One session carrying what the player accessors are read through: a pebble created and destroyed
// part way in, and three queries on one step, two of them tagged. A record written before a step
// belongs to the frame that step completes, so each lands one frame after the step it precedes.
fn bool a_busy_session_recorded_into(DroppedPair pair, b3::Recording* recording) {
    if (catch pair.world.start_recording(recording)) return false;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere pebble = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.25 };
    b3::Pos from_above = { 0.0, 3.0, 0.0 };
    b3::Vec3 straight_down = { 0.0, -5.0, 0.0 };
    b3::ShapeId[8] overlapped;
    b3::BodyId transient;

    for (int step = 0; step < A_BUSY_FRAME_COUNT; step++) {
        if (step == A_PEBBLE_STEP) {
            b3::BodyDef pebble_def = b3::default_body_def();
            pebble_def.type = b3::BodyType.DYNAMIC;
            pebble_def.position = { 0.0, 4.0, 0.0 };
            transient = pair.world.create_body(&pebble_def)!!;
            transient.create_sphere_shape(&shape_def, &pebble)!!;
        }
        if (step == A_QUERY_STEP) {
            b3::QueryFilter ray_filter = b3::default_query_filter();
            ray_filter.id = A_TAGGED_RAY_ID;
            ray_filter.name = "a_dropped_ray";
            if (catch pair.world.cast_ray_closest(from_above, straight_down, ray_filter)) return false;

            b3::QueryFilter box_filter = b3::default_query_filter();
            box_filter.id = A_TAGGED_BOX_ID;
            box_filter.name = "a_dropped_box";
            b3::Aabb around_them = { .lower_bound = { -6.0, -2.0, -6.0 }, .upper_bound = { 6.0, 6.0, 6.0 } };
            b3::ShapeId[] found = pair.world.overlap_aabb_all(around_them, box_filter, overlapped[..]);
            if (found.len == 0) return false;

            if (catch pair.world.cast_ray_closest(from_above, straight_down, b3::default_query_filter())) return false;
        }
        if (step == A_PEBBLE_END_STEP) transient.destroy();
        pair.world.step(A_TIME_STEP, A_SUB_STEP_COUNT);
    }
    pair.world.stop_recording();
    return true;
}

struct ReplayEnds {
    bool the_stream_was_live_at_the_open;
    bool the_pause_was_clear_at_the_open;
    bool the_stream_ran_out_at_the_end;
    int frame_at_the_end;
    bool a_step_past_the_end_was_refused;
    int bodies_before_the_pause;
    int bodies_at_the_pause;
    bool the_sub_step_paused_before_the_step;
    int frame_at_the_pause;
    int frame_after_the_pause;
    bool the_pause_cleared_on_the_step;
}

fn ReplayEnds the_readings_at_both_ends_of_a_replay() {
    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    ReplayEnds ends;
    if (!a_busy_session_recorded_into(pair, recording)) return ends;

    b3::RecPlayer*? player = b3::create_player(recording.data(), 1);
    if (catch player) return ends;

    ends.the_stream_was_live_at_the_open = !player.is_at_end();
    ends.the_pause_was_clear_at_the_open = !player.is_at_pre_step();

    while (player.step_frame()) {}
    ends.the_stream_ran_out_at_the_end = player.is_at_end();
    ends.frame_at_the_end = player.frame();
    ends.a_step_past_the_end_was_refused = !player.step_frame();

    player.seek_frame(A_PEBBLE_STEP);
    ends.bodies_before_the_pause = player.body_count();
    player.sub_step_frame();
    ends.the_sub_step_paused_before_the_step = player.is_at_pre_step();
    ends.frame_at_the_pause = player.frame();
    ends.bodies_at_the_pause = player.body_count();
    player.sub_step_frame();
    ends.frame_after_the_pause = player.frame();
    ends.the_pause_cleared_on_the_step = !player.is_at_pre_step();

    player.destroy();
    return ends;
}

fn void a_replay_reports_both_its_ends_and_pauses_where_a_body_is_created() @test {
    ReplayEnds ends = the_readings_at_both_ends_of_a_replay();

    assert(ends.the_stream_was_live_at_the_open && ends.the_pause_was_clear_at_the_open,
        "a player opens with the whole stream ahead of it and no pause outstanding");
    assert(ends.frame_at_the_end == A_BUSY_FRAME_COUNT && ends.the_stream_ran_out_at_the_end
        && ends.a_step_past_the_end_was_refused,
        "and reports the end once the last recorded frame has been replayed");
    assert(ends.frame_at_the_pause == A_PEBBLE_STEP && ends.the_sub_step_paused_before_the_step,
        "a sub step over a frame that creates a body stops before that frame's step");
    assert(ends.bodies_before_the_pause == A_PEBBLE_ORDINAL
        && ends.bodies_at_the_pause == A_PEBBLE_ORDINAL + 1,
        "with the pebble already created, which is what makes the pause worth drawing");
    assert(ends.frame_after_the_pause == A_PEBBLE_STEP + 1 && ends.the_pause_cleared_on_the_step,
        "and the second sub step runs that step and clears the pause");
}

struct BodyOrdinals {
    int bodies_before_the_pebble;
    int bodies_after_the_pebble;
    int bodies_after_the_pebble_was_destroyed;
    bool the_pebble_was_live_while_it_lasted;
    fault the_hole_it_left;
    fault past_the_last_ordinal;
    fault below_the_first_ordinal;
}

fn BodyOrdinals the_readings_across_a_destroyed_body_ordinal() {
    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    BodyOrdinals ordinals;
    if (!a_busy_session_recorded_into(pair, recording)) return ordinals;

    b3::RecPlayer*? player = b3::create_player(recording.data(), 1);
    if (catch player) return ordinals;

    player.seek_frame(A_PEBBLE_STEP);
    ordinals.bodies_before_the_pebble = player.body_count();

    player.seek_frame(A_PEBBLE_END_STEP);
    ordinals.bodies_after_the_pebble = player.body_count();
    if (try pebble = player.body_id(A_PEBBLE_ORDINAL)) {
        ordinals.the_pebble_was_live_while_it_lasted = pebble.is_valid();
    }

    player.seek_frame(A_PEBBLE_END_STEP + 1);
    ordinals.bodies_after_the_pebble_was_destroyed = player.body_count();
    if (catch err = player.body_id(A_PEBBLE_ORDINAL)) ordinals.the_hole_it_left = err;
    if (catch err = player.body_id(player.body_count())) ordinals.past_the_last_ordinal = err;
    if (catch err = player.body_id(-1)) ordinals.below_the_first_ordinal = err;

    player.destroy();
    return ordinals;
}

fn void a_destroyed_body_leaves_its_ordinal_behind_as_a_hole() @test {
    BodyOrdinals ordinals = the_readings_across_a_destroyed_body_ordinal();

    assert(ordinals.bodies_before_the_pebble == A_PEBBLE_ORDINAL
        && ordinals.bodies_after_the_pebble == A_PEBBLE_ORDINAL + 1,
        "the pebble the session created part way in takes the next creation ordinal");
    assert(ordinals.the_pebble_was_live_while_it_lasted,
        "which names a live body for as long as the recording keeps it");
    assert(ordinals.bodies_after_the_pebble_was_destroyed == A_PEBBLE_ORDINAL + 1
        && ordinals.the_hole_it_left == b3::INVALID_HANDLE,
        "and stays counted once it is destroyed, naming nothing from there on");
    assert(ordinals.past_the_last_ordinal == b3::INDEX_OUT_OF_RANGE
        && ordinals.below_the_first_ordinal == b3::INDEX_OUT_OF_RANGE,
        "where an ordinal the replay never handed out is refused instead");
}

const usz A_KEYFRAME_BUDGET = 1048576;
const int A_KEYFRAME_INTERVAL = 8;

struct KeyframePolicy {
    usz budget_at_the_open;
    int min_interval_at_the_open;
    int interval_at_the_open;
    usz bytes_after_a_replay;
    usz budget_after_the_change;
    int min_interval_after_the_change;
    int interval_after_the_change;
    usz bytes_after_the_change;
    usz budget_after_a_keeping_call;
    int min_interval_after_a_keeping_call;
}

fn KeyframePolicy the_readings_around_a_keyframe_policy() {
    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    KeyframePolicy policy;
    if (!a_busy_session_recorded_into(pair, recording)) return policy;

    b3::RecPlayer*? player = b3::create_player(recording.data(), 1);
    if (catch player) return policy;

    policy.budget_at_the_open = player.keyframe_budget();
    policy.min_interval_at_the_open = player.keyframe_min_interval();
    policy.interval_at_the_open = player.keyframe_interval();

    while (player.step_frame()) {}
    policy.bytes_after_a_replay = player.keyframe_bytes();

    player.set_keyframe_policy(A_KEYFRAME_BUDGET, A_KEYFRAME_INTERVAL);
    policy.budget_after_the_change = player.keyframe_budget();
    policy.min_interval_after_the_change = player.keyframe_min_interval();
    policy.interval_after_the_change = player.keyframe_interval();
    policy.bytes_after_the_change = player.keyframe_bytes();

    player.set_keyframe_policy(0, -A_KEYFRAME_INTERVAL);
    policy.budget_after_a_keeping_call = player.keyframe_budget();
    policy.min_interval_after_a_keeping_call = player.keyframe_min_interval();

    player.destroy();
    return policy;
}

fn void a_keyframe_policy_round_trips_and_drops_the_ring_it_replaces() @test {
    KeyframePolicy policy = the_readings_around_a_keyframe_policy();

    assert(policy.budget_at_the_open >= 268435456 && policy.budget_at_the_open <= 1073741824,
        "a player opens with a keyframe budget of a few hundred megabytes");
    assert(policy.min_interval_at_the_open > 1 && policy.min_interval_at_the_open < 128
        && policy.interval_at_the_open == policy.min_interval_at_the_open,
        "and captures at its finest spacing until the ring has to evict");
    assert(policy.bytes_after_a_replay > 0,
        "replaying to the end fills the ring, which is the memory the budget caps");
    assert(policy.budget_after_the_change == A_KEYFRAME_BUDGET
        && policy.min_interval_after_the_change == A_KEYFRAME_INTERVAL
        && policy.interval_after_the_change == A_KEYFRAME_INTERVAL,
        "setting a policy takes both values and restarts the spacing at the new finest one");
    assert(policy.bytes_after_the_change == 0 && policy.bytes_after_a_replay > 0,
        "and drops the snapshots the old policy had captured");
    assert(policy.budget_after_a_keeping_call == A_KEYFRAME_BUDGET
        && policy.min_interval_after_a_keeping_call == A_KEYFRAME_INTERVAL,
        "where a budget of zero and an interval below one keep the policy rather than clearing it");
}

struct WorkerCounts {
    int at_the_open;
    int in_the_replay_world;
    int above_the_ceiling;
    int in_the_world_above_the_ceiling;
    int at_zero;
    int below_zero;
    int back_at_one;
}

fn WorkerCounts the_readings_around_a_worker_count() {
    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    WorkerCounts counts;
    if (!a_busy_session_recorded_into(pair, recording)) return counts;

    b3::RecPlayer*? player = b3::create_player(recording.data(), 1);
    if (catch player) return counts;

    counts.at_the_open = player.info().worker_count;
    counts.in_the_replay_world = player.world_id().worker_count();

    player.set_worker_count(4 * b3::MAX_WORKERS);
    counts.above_the_ceiling = player.info().worker_count;
    counts.in_the_world_above_the_ceiling = player.world_id().worker_count();

    player.set_worker_count(0);
    counts.at_zero = player.info().worker_count;
    player.set_worker_count(-b3::MAX_WORKERS);
    counts.below_zero = player.info().worker_count;
    player.set_worker_count(1);
    counts.back_at_one = player.info().worker_count;

    player.destroy();
    return counts;
}

fn void a_worker_count_outside_the_range_is_clamped_rather_than_refused() @test {
    WorkerCounts counts = the_readings_around_a_worker_count();

    assert(counts.at_the_open == 1 && counts.in_the_replay_world == 1,
        "a player opens at the count it was created with, and stands its world up on it");
    assert(counts.above_the_ceiling == b3::MAX_WORKERS
        && counts.in_the_world_above_the_ceiling == b3::MAX_WORKERS,
        "a count above the ceiling lands on it, in the player and in the world it drives");
    assert(counts.at_zero == 1 && counts.below_zero == 1,
        "and zero and a negative land on one rather than being refused");
    assert(counts.back_at_one == 1 && counts.above_the_ceiling > counts.back_at_one,
        "a count inside the range is taken as it stands");
}

const int A_QUERY_FRAME = A_QUERY_STEP + 1;
const int A_RECORDED_QUERY_COUNT = 3;

struct FrameQueries {
    int at_the_frame_before;
    int at_the_query_frame;
    int at_the_frame_after;
    b3::RecQueryType the_rays_kind;
    b3::RecQueryType the_boxs_kind;
    ulong the_rays_id;
    ulong the_boxs_id;
    bool the_ray_named_itself;
    bool the_box_named_itself;
    bool the_untagged_ray_carried_no_tag;
    int the_rays_hits;
    int the_boxs_hits;
    b3::Pos the_rays_origin;
    b3::Vec3 the_rays_translation;
    b3::Aabb the_boxs_bounds;
    float the_rays_hit_fraction;
    float the_rays_hit_normal_y;
    bool every_box_hit_named_a_shape;
    fault past_the_last_query;
    fault below_the_first_query;
    fault past_a_querys_hits;
    fault a_hit_of_no_query;
}

fn FrameQueries the_readings_from_a_replayed_frames_queries() {
    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    FrameQueries queries;
    if (!a_busy_session_recorded_into(pair, recording)) return queries;

    b3::RecPlayer*? player = b3::create_player(recording.data(), 1);
    if (catch player) return queries;

    player.seek_frame(A_QUERY_FRAME - 1);
    queries.at_the_frame_before = player.frame_query_count();
    player.seek_frame(A_QUERY_FRAME);
    queries.at_the_query_frame = player.frame_query_count();

    if (try ray = player.frame_query(0)) {
        queries.the_rays_kind = ray.type;
        queries.the_rays_id = ray.id;
        queries.the_ray_named_itself = ray.name != null && ray.name.str_view() == "a_dropped_ray";
        queries.the_rays_hits = ray.hit_count;
        queries.the_rays_origin = ray.origin;
        queries.the_rays_translation = ray.translation;
    }
    if (try hit = player.frame_query_hit(0, 0)) {
        queries.the_rays_hit_fraction = hit.fraction;
        queries.the_rays_hit_normal_y = hit.normal.y;
    }
    if (try box = player.frame_query(1)) {
        queries.the_boxs_kind = box.type;
        queries.the_boxs_id = box.id;
        queries.the_box_named_itself = box.name != null && box.name.str_view() == "a_dropped_box";
        queries.the_boxs_hits = box.hit_count;
        queries.the_boxs_bounds = box.aabb;

        bool named = box.hit_count > 0;
        for (int index = 0; index < box.hit_count; index++) {
            b3::RecQueryHit? hit = player.frame_query_hit(1, index);
            if (catch hit) {
                named = false;
                continue;
            }
            if (!hit.shape.is_valid()) named = false;
        }
        queries.every_box_hit_named_a_shape = named;
    }
    if (try untagged = player.frame_query(2)) {
        queries.the_untagged_ray_carried_no_tag = untagged.key == 0 && untagged.id == 0 && untagged.name == null;
    }

    if (catch err = player.frame_query(player.frame_query_count())) queries.past_the_last_query = err;
    if (catch err = player.frame_query(-1)) queries.below_the_first_query = err;
    if (catch err = player.frame_query_hit(0, queries.the_rays_hits)) queries.past_a_querys_hits = err;
    if (catch err = player.frame_query_hit(player.frame_query_count(), 0)) queries.a_hit_of_no_query = err;

    player.seek_frame(A_QUERY_FRAME + 1);
    queries.at_the_frame_after = player.frame_query_count();

    player.destroy();
    return queries;
}

fn void a_replayed_frame_reports_the_queries_recorded_ahead_of_its_step() @test {
    FrameQueries queries = the_readings_from_a_replayed_frames_queries();

    assert(queries.at_the_query_frame == A_RECORDED_QUERY_COUNT && queries.at_the_frame_before == 0
        && queries.at_the_frame_after == 0,
        "the three queries belong to the frame whose step they were recorded ahead of, and to no other");
    assert(queries.the_rays_kind == b3::RecQueryType.CAST_RAY_CLOSEST
        && queries.the_boxs_kind == b3::RecQueryType.OVERLAP_AABB,
        "each names the world call that made it");
    assert(queries.the_rays_id == A_TAGGED_RAY_ID && queries.the_boxs_id == A_TAGGED_BOX_ID
        && queries.the_ray_named_itself && queries.the_box_named_itself,
        "a tagged query carries back the id and name its filter was given");
    assert(queries.the_untagged_ray_carried_no_tag,
        "and an untagged one carries no key, no id and no name at all");
    assert(queries.the_rays_origin.y > 2.95 && queries.the_rays_origin.y < 3.05
        && queries.the_rays_translation.y > -5.05 && queries.the_rays_translation.y < -4.95,
        "the ray reports the span it was fired over rather than a normalised direction");
    assert(queries.the_boxs_bounds.lower_bound.y > -2.05 && queries.the_boxs_bounds.lower_bound.y < -1.95
        && queries.the_boxs_bounds.upper_bound.y > 5.95 && queries.the_boxs_bounds.upper_bound.y < 6.05,
        "and the overlap reports the box it was given");
    assert(queries.the_rays_hits == 1 && queries.the_rays_hit_fraction > 0.59
        && queries.the_rays_hit_fraction < 0.61 && queries.the_rays_hit_normal_y > 0.95,
        "a cast records where its hit landed, three fifths of the way down onto the slab's top face");
    assert(queries.the_boxs_hits == A_DROPPED_BODY_COUNT + 1 && queries.every_box_hit_named_a_shape,
        "where an overlap records the shapes alone, one per body in the busy session");
    assert(queries.past_the_last_query == b3::INDEX_OUT_OF_RANGE
        && queries.below_the_first_query == b3::INDEX_OUT_OF_RANGE,
        "an index no query holds is refused rather than answered with a zeroed struct");
    assert(queries.past_a_querys_hits == b3::INDEX_OUT_OF_RANGE
        && queries.a_hit_of_no_query == b3::INDEX_OUT_OF_RANGE,
        "and so is a hit past a query's own count, or one of a query that is not there");
}

struct DebugShapeTally {
    int offered;
    int withdrawn;
}

fn void* offer_a_debug_shape(void* debug_shape, void* user_context) {
    DebugShapeTally* tally = (DebugShapeTally*)user_context;
    tally.offered++;
    return user_context;
}

fn void withdraw_a_debug_shape(void* user_shape, void* user_context) {
    DebugShapeTally* tally = (DebugShapeTally*)user_context;
    tally.withdrawn++;
}

struct DebugShapeWiring {
    ushort generation_before;
    ushort generation_after;
    bool the_world_it_replaced_stopped_being_valid;
    bool the_world_it_stood_up_is_valid;
    int frame_before;
    int frame_after;
    int worlds_before;
    int worlds_after;
    int bodies_at_the_end;
    int shapes_offered_by_a_replay;
    int shapes_withdrawn_by_a_replay;
}

fn DebugShapeWiring the_readings_around_wiring_a_renderer_in() {
    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    DebugShapeWiring wiring;
    if (!a_busy_session_recorded_into(pair, recording)) return wiring;

    b3::RecPlayer*? player = b3::create_player(recording.data(), 1);
    if (catch player) return wiring;

    player.seek_frame(A_QUERY_FRAME);
    b3::WorldId replaced = player.world_id();
    wiring.generation_before = replaced.generation;
    wiring.frame_before = player.frame();
    wiring.worlds_before = b3::world_count();

    DebugShapeTally tally;
    player.set_debug_shape_callbacks(&offer_a_debug_shape, &withdraw_a_debug_shape, &tally);

    b3::WorldId stood_up = player.world_id();
    wiring.generation_after = stood_up.generation;
    wiring.the_world_it_replaced_stopped_being_valid = !replaced.is_valid();
    wiring.the_world_it_stood_up_is_valid = stood_up.is_valid();
    wiring.frame_after = player.frame();
    wiring.worlds_after = b3::world_count();

    while (player.step_frame()) {}
    wiring.bodies_at_the_end = player.body_count();
    wiring.shapes_offered_by_a_replay = tally.offered;
    wiring.shapes_withdrawn_by_a_replay = tally.withdrawn;

    player.destroy();
    return wiring;
}

fn void wiring_a_renderer_in_stands_a_new_replay_world_up() @test {
    DebugShapeWiring wiring = the_readings_around_wiring_a_renderer_in();

    assert(wiring.frame_before == A_QUERY_FRAME && wiring.frame_after == 0,
        "wiring the callbacks in rewinds the replay to frame 0, wherever it had reached");
    assert(wiring.generation_after != wiring.generation_before
        && wiring.the_world_it_replaced_stopped_being_valid && wiring.the_world_it_stood_up_is_valid,
        "and stands a new world up in place of the one whose id the caller was holding");
    assert(wiring.worlds_before > 0 && wiring.worlds_after == wiring.worlds_before,
        "one world for one, so the rebuild leaks none");
    assert(wiring.bodies_at_the_end == A_PEBBLE_ORDINAL + 1
        && wiring.shapes_offered_by_a_replay == 0 && wiring.shapes_withdrawn_by_a_replay == 0,
        "and replaying every record afterwards reaches neither callback: only a draw of that world does");
}
