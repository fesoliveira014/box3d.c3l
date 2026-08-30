module b3;

<*
 Adds a body to this world.

 The definition must come from default_body_def(). box3d marks a definition with an internal cookie
 there and checks it with an assertion, but this package links a release build where that assertion
 is compiled out — an unmarked definition would be accepted and would produce a static body at the
 origin regardless of what the caller intended. That cookie check is the only one this guard
 restores: box3d also asserts a valid position, quaternion, velocities, and non-negative damping on
 the definition, and those assertions are compiled out here too and none of them are restored, so a
 caller handing in, say, a zero quaternion is still accepted silently. This is a known limit of the
 guard, not an oversight.

 @param [&in] def : "a definition obtained from default_body_def()"
 @return "the body, INVALID_DEFINITION if the definition did not come from default_body_def(), or INVALID_HANDLE if box3d returns a null identifier — which b3CreateBody does only when the target world is locked, mid b3World_Step or inside a callback box3d runs during one"
*>
fn BodyId? WorldId.create_body(self, BodyDef* def) {
    if (def.internal_value != default_body_def().internal_value) return INVALID_DEFINITION~;
    return create_body_raw(self, def).checked();
}
