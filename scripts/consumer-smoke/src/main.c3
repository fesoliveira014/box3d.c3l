module consumer;

import b3;

// The release gate: what a consumer holding nothing but b3.c3l can build. It is a real .c3 file
// rather than a heredoc inside the workflow so that a sweep over the package's own sources reaches
// it -- the API change that made it stale was applied to src/ and test/ and could not see YAML.
fn int main() {
    b3::require_float_abi()!!;

    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer b3::destroy_world(world);
    b3::world_step(world, 1.0f / 60.0f, 4);

    draw::DebugDraw debug = draw::default_debug_draw();
    draw::world_draw(world, &debug, ulong::max);

    record::Recording* recording = record::create_recording(4096);
    defer record::destroy_recording(recording);
    record::world_start_recording(world, recording)!!;
    b3::world_step(world, 1.0f / 60.0f, 4);
    record::world_stop_recording(world);

    return 0;
}
