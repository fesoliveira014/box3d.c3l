module b3;

<*
 Creates a simulation world.

 The definition must come from default_world_def(). box3d marks a definition with an internal cookie
 there and checks it with an assertion, but this package links a release build where that assertion
 is compiled out — an unmarked definition would be accepted and would simulate with zero gravity and
 zero contact stiffness. The check below restores it.

 @param [&in] def : "a definition obtained from default_world_def()"
 @return "the world, INVALID_DEFINITION if the definition did not come from default_world_def(), or WORLD_LIMIT_REACHED when MAX_WORLDS worlds already exist"
*>
fn WorldId? create_world(WorldDef* def) {
    if (def.internal_value != default_world_def().internal_value) return INVALID_DEFINITION~;
    return create_world_raw(def).checked();
}
