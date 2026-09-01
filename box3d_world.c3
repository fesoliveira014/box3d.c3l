module b3;

<*
 Creates a simulation world.

 The definition must come from default_world_def(): this release build compiles box3d's cookie
 assertion out, so an unmarked one is accepted silently and simulates with zero gravity and zero
 contact stiffness.

 @param [&in] def : "a definition obtained from default_world_def()"
 @return "the world, INVALID_DEFINITION if the definition did not come from default_world_def(), or WORLD_LIMIT_REACHED when MAX_WORLDS worlds already exist"
*>
fn WorldId? create_world(WorldDef* def) {
    if (def.internal_value != default_world_def().internal_value) return INVALID_DEFINITION~;
    WorldId world = create_world_raw(def);
    // A null here always means the world limit, never the stale handle WorldId.checked() would
    // report as INVALID_HANDLE.
    if (world.is_null()) return WORLD_LIMIT_REACHED~;
    return world;
}
