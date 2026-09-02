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
