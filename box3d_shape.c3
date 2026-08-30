module b3;

<*
 Attaches a sphere to this body.

 The definition must come from default_shape_def(). box3d marks a definition with an internal
 cookie there and checks it with an assertion this release build compiles out, so an unmarked
 definition would otherwise be accepted and would produce a shape of zero density that
 contributes no mass to the body. That cookie check is the only one this guard restores; the
 assertions box3d makes on density and material coefficients are compiled out here too and
 none of them are restored.

 @param [&in] def : "a definition obtained from default_shape_def()"
 @param [&in] sphere : "the sphere in body-local space; box3d clones it"
 @return "the shape, INVALID_DEFINITION if the definition did not come from default_shape_def(), or INVALID_HANDLE if box3d returns a null identifier"
*>
fn ShapeId? BodyId.create_sphere_shape(self, ShapeDef* def, Sphere* sphere) {
    if (def.internal_value != default_shape_def().internal_value) return INVALID_DEFINITION~;
    return create_sphere_shape_raw(self, def, sphere).checked();
}

<*
 Attaches a capsule to this body. The definition must come from default_shape_def() — see
 create_sphere_shape. box3d substitutes a sphere when the two centres fall within its linear
 slop of each other, so the returned shape's type() may be SPHERE rather than CAPSULE.

 @param [&in] def : "a definition obtained from default_shape_def()"
 @param [&in] capsule : "the capsule in body-local space; box3d clones it"
 @return "the shape, INVALID_DEFINITION if the definition did not come from default_shape_def(), or INVALID_HANDLE if box3d returns a null identifier"
*>
fn ShapeId? BodyId.create_capsule_shape(self, ShapeDef* def, Capsule* capsule) {
    if (def.internal_value != default_shape_def().internal_value) return INVALID_DEFINITION~;
    return create_capsule_shape_raw(self, def, capsule).checked();
}

<*
 Attaches a convex hull to this body. The definition must come from default_shape_def() — see
 create_sphere_shape. box3d clones the hull, so a stack BoxHull is enough and nothing needs to
 outlive the call.

 @param [&in] def : "a definition obtained from default_shape_def()"
 @param [&in] hull : "the hull in body-local space, usually the base of a BoxHull"
 @return "the shape, INVALID_DEFINITION if the definition did not come from default_shape_def(), or INVALID_HANDLE if box3d returns a null identifier"
*>
fn ShapeId? BodyId.create_hull_shape(self, ShapeDef* def, HullData* hull) {
    if (def.internal_value != default_shape_def().internal_value) return INVALID_DEFINITION~;
    return create_hull_shape_raw(self, def, hull).checked();
}

<*
 Attaches a convex hull to this body with a scale and a local transform baked into it. Scale
 applies before the transform and may be non-uniform or mirrored. The definition must come from
 default_shape_def() — see create_sphere_shape.

 @param [&in] def : "a definition obtained from default_shape_def()"
 @param [&in] hull : "the hull to bake, usually the base of a BoxHull"
 @return "the shape, INVALID_DEFINITION if the definition did not come from default_shape_def(), or INVALID_HANDLE if box3d returns a null identifier"
*>
fn ShapeId? BodyId.create_transformed_hull_shape(self, ShapeDef* def, HullData* hull, Transform transform, Vec3 scale) {
    if (def.internal_value != default_shape_def().internal_value) return INVALID_DEFINITION~;
    return create_transformed_hull_shape_raw(
      body_id: self,
      def: def,
      hull: hull,
      transform: transform,
      scale: scale,
    ).checked();
}

<*
 @return "the sphere geometry, or WRONG_SHAPE_TYPE when this shape is not a sphere"
*>
fn Sphere? ShapeId.sphere(self) {
    if (self.type() != ShapeType.SPHERE) return WRONG_SHAPE_TYPE~;
    return sphere_raw(self);
}

<*
 @return "the capsule geometry, or WRONG_SHAPE_TYPE when this shape is not a capsule"
*>
fn Capsule? ShapeId.capsule(self) {
    if (self.type() != ShapeType.CAPSULE) return WRONG_SHAPE_TYPE~;
    return capsule_raw(self);
}

<*
 The hull belongs to the world and stays valid until the shape is destroyed or given other
 geometry.

 @return "the hull, or WRONG_SHAPE_TYPE when this shape is not a hull"
*>
fn HullData*? ShapeId.hull(self) {
    if (self.type() != ShapeType.HULL) return WRONG_SHAPE_TYPE~;
    return hull_raw(self);
}
