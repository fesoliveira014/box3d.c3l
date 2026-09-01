module b3;

<*
 Adds a body to this world.

 The definition must come from default_body_def(): this release build compiles box3d's cookie
 assertion out, so an unmarked one is accepted silently and yields a static body at the origin.
 The cookie is the only compiled-out assertion this guard restores — a zero quaternion, a bad
 position or negative damping still passes.

 @param [&in] def : "a definition obtained from default_body_def()"
 @return "the body, INVALID_DEFINITION if the definition did not come from default_body_def(), or INVALID_HANDLE if box3d returns a null identifier — which b3CreateBody does only when the target world is locked, mid b3World_Step or inside a callback box3d runs during one"
*>
fn BodyId? WorldId.create_body(self, BodyDef* def) {
    if (def.internal_value != default_body_def().internal_value) return INVALID_DEFINITION~;
    return create_body_raw(self, def).checked();
}
