module b3::record;

<*
 The bytes recorded so far, empty until a session has written into it. The slice borrows box3d's
 buffer, which moves as it grows, so a recorded step invalidates every slice taken before it, as
 destroy() does. box3d's header calls the pointer null before the first write; it is not, and the
 count is what says a recording is empty.
*>
fn char[] Recording.data(&self) {
    return recording_data_raw(self)[:recording_size_raw(self)];
}

<*
 Starts recording this world's mutations, resetting the buffer so one recording serves session
 after session. box3d refuses in silence — a locked world, a null recording, a world already
 recording — and writes its header when it accepts, which is what the fault reads.

 @param recording : `the buffer to write into, which must outlive the session`
 @return? b3::NOT_A_RECORDING : `the buffer holds nothing after the call — so any buffer that is not
   empty when box3d refuses reads as accepted, including the one this world is already recording
   into and one just loaded from a file`
*>
fn void? world_start_recording(WorldId world, Recording* recording) {
    if (recording == null) return b3::NOT_A_RECORDING~;
    world_start_recording_raw(world, recording);
    if (recording_size_raw(recording) == 0) return b3::NOT_A_RECORDING~;
}

<*
 Writes the recorded bytes to the path, replacing whatever is there. A recording no session has
 filled writes a well-formed empty file that load_recording() then refuses, so a round trip only
 survives bytes a session actually produced.

 @return? b3::FILE_UNAVAILABLE : `box3d could not open the path, or the write fell short`
*>
fn void? save_recording_to_file(Recording* recording, ZString path) {
    if (!save_recording_to_file_raw(recording, path)) return b3::FILE_UNAVAILABLE~;
}

<*
 Reads back what save() wrote. box3d checks the file's magic here, so a file that is no recording
 is refused at the load rather than at create_player(), and every refusal is allocation-free. The
 caller owns the result and must destroy() it — see Recording.

 @return `the recording`
 @return? b3::FILE_UNAVAILABLE : `the path could not be read, or carries no recording, which box3d
   does not distinguish`
*>
fn Recording*? load_recording(ZString path) {
    Recording* recording = load_recording_from_file_raw(path);
    if (recording == null) return b3::FILE_UNAVAILABLE~;
    return recording;
}

<*
 Replays the bytes in a world of its own and checks every state hash they carry, which is what
 makes a recording a determinism test. It creates and destroys that world itself, leaving
 world_count() and byte_count() where it found them — but not the process-wide length unit, which
 a buffer refused past its header leaves at the recording's value with both counters clean.

 @param worker_count : `the count it stands its replay world up with, which it hands to
   create_player()`
 @return `true when the replay reproduced every recorded hash — false covers both a divergence and
   bytes box3d could not read as a recording`
*>
fn bool validate_replay(char[] data, int worker_count) {
    return validate_replay_raw(data.ptr, (int)data.len, worker_count);
}

<*
 Stands up a replay of these bytes — see Player for the world it owns. box3d copies them, so
 the slice need not outlive the call, and it prints on standard output the reason it refused one.
 A live player holds the process-wide length unit at the one its recording was made under, and
 only a player that was built gives it back: a buffer refused past its header keeps it, silently,
 with world_count() and byte_count() both correct.

 @param worker_count : `how many workers the replay world runs, clamped into box3d's worker range;
   the recording does not carry the count it was produced at, so replaying under a different one
   is the caller's own cross-thread determinism test`
 @return `the player`
 @return? b3::NOT_A_RECORDING : `the bytes carry no header box3d will replay`
*>
fn Player*? create_player(char[] data, int worker_count) {
    Player* player = create_player_raw(data.ptr, (int)data.len, worker_count);
    if (player == null) return b3::NOT_A_RECORDING~;
    return player;
}

<*
 Where the replay first stopped reproducing the recording, which is the frame to look at when
 has_diverged() answers true.

 @return `the frame`
 @return? b3::NO_HIT : `every state hash has matched — restart() puts it back to that`
*>
fn int? Player.diverge_frame(&self) {
    int frame = diverge_frame_raw(self);
    if (frame < 0) return b3::NO_HIT~;
    return frame;
}

<*
 The live body at a creation ordinal — see body_count() for why an ordinal below the count is not
 a promise of a body.

 @return `the body`
 @return? b3::INDEX_OUT_OF_RANGE : `index is outside 0 to body_count() - 1`
 @return? b3::INVALID_HANDLE : `the body at that ordinal has been destroyed`
*>
fn BodyId? Player.body_id(&self, int index) {
    if (index < 0 || index >= self.body_count()) return b3::INDEX_OUT_OF_RANGE~;
    return body_id_raw(self, index).checked();
}

<*
 One of the queries the last replayed frame re-issued. box3d answers an index it does not hold
 with a zeroed struct, which reads as a real untagged overlap of nothing, hence the fault.

 @return `the query`
 @return? b3::INDEX_OUT_OF_RANGE : `index is outside 0 to frame_query_count() - 1`
*>
fn QueryInfo? Player.frame_query(&self, int index) {
    if (index < 0 || index >= self.frame_query_count()) return b3::INDEX_OUT_OF_RANGE~;
    return frame_query_raw(self, index);
}

<*
 One result of one of the last replayed frame's queries — see QueryHit for the fields an
 overlap leaves holding the replay's scratch rather than a hit.

 @return `the hit`
 @return? Player.frame_query!
 @return? b3::INDEX_OUT_OF_RANGE : `that query holds no hit_index`
*>
fn QueryHit? Player.frame_query_hit(&self, int query_index, int hit_index) {
    QueryInfo query = self.frame_query(query_index)!;
    if (hit_index < 0 || hit_index >= query.hit_count) return b3::INDEX_OUT_OF_RANGE~;
    return frame_query_hit_raw(self, query_index, hit_index);
}
