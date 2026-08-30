module b3;

<*
 Adds a body to this world.

 The definition must come from default_body_def(). box3d marks a definition with an internal cookie
 there and checks it with an assertion, but this package links a release build where that assertion
 is compiled out — an unmarked definition would be accepted and would produce a static body at the
 origin regardless of what the caller intended.

 @param [&in] def : "a definition obtained from default_body_def()"
 @return "the body, INVALID_DEFINITION if the definition did not come from default_body_def(), or INVALID_HANDLE if box3d returned nothing"
*>
fn BodyId? WorldId.create_body(self, BodyDef* def) {
    if (def.internal_value != default_body_def().internal_value) return INVALID_DEFINITION~;
    return create_body_raw(self, def).checked();
}
