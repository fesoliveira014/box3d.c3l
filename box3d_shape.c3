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

<*
 The mesh belongs to the caller rather than to the world: data is the very pointer
 BodyId.create_mesh_shape() or set_mesh() was handed, and it must still outlive this shape.

 @return "the mesh and the scale it is read through, or WRONG_SHAPE_TYPE when this shape is not a mesh"
*>
fn Mesh? ShapeId.mesh(self) {
    if (self.type() != ShapeType.MESH) return WRONG_SHAPE_TYPE~;
    return mesh_raw(self);
}

<*
 The height field belongs to the caller rather than to the world — see mesh() — and it too must
 outlive this shape.

 @return "the height field, or WRONG_SHAPE_TYPE when this shape is not a height field"
*>
fn HeightFieldData*? ShapeId.height_field(self) {
    if (self.type() != ShapeType.HEIGHT_FIELD) return WRONG_SHAPE_TYPE~;
    return height_field_raw(self);
}

<*
 Gives this shape a mesh, replacing whatever geometry it had and making its type MESH. There is
 no type guard here, unlike on mesh(): changing the type is what the call is for.

 The shape borrows the new mesh exactly as BodyId.create_mesh_shape() does, and nothing happens
 to the old one. box3d drops that pointer without freeing it — byte_count() does not move across
 the call — so a caller replacing a mesh is left holding two, and owes destroy() on both.

 The body's mass is not recomputed. It keeps whatever the previous geometry gave it until
 BodyId.apply_mass_from_shapes() is called, and a mesh contributes none, so that call takes a
 body carrying nothing else to zero mass.

 Silently does nothing while the world is locked, mid WorldId.step or inside a callback box3d
 runs during one.

 @param mesh : "the mesh, which must outlive the shape; null is refused rather than dereferenced"
 @param scale : "how far the mesh's vertices are stretched along each axis, non-uniform or mirrored; a zero component is pushed away from zero rather than refused"
 @return "void, or INVALID_DEFINITION when the mesh is null, which is what create_mesh_shape answers there too"
*>
fn void? ShapeId.set_mesh(self, MeshData* mesh, Vec3 scale) {
    if (mesh == null) return INVALID_DEFINITION~;
    set_mesh_raw(self, mesh, scale);
}

<*
 One of the surface materials this shape holds, by index. There is no type guard — what a shape
 holds here has nothing to do with the geometry it carries — but there is an index guard, because
 the bounds assertion box3d makes is compiled out here and an index outside the array reads
 whatever follows it in memory. A shape built from an empty compound holds no materials at all,
 and every index into it is refused.

 @return "the material, or INDEX_OUT_OF_RANGE when index is negative or not below mesh_material_count()"
*>
fn SurfaceMaterial? ShapeId.mesh_surface_material(self, int index) {
    if (index < 0 || index >= self.mesh_material_count()) return INDEX_OUT_OF_RANGE~;
    return mesh_surface_material_raw(self, index);
}

<*
 Replaces one of the surface materials this shape holds. Index 0 is the same material
 set_surface_material() writes and friction() and restitution() read, so writing it is visible
 through all three. The index guard is the one mesh_surface_material() carries, and here what it
 stops is a write past the end of the array rather than a read.

 @return "void, or INDEX_OUT_OF_RANGE when index is negative or not below mesh_material_count()"
*>
fn void? ShapeId.set_mesh_material(self, SurfaceMaterial material, int index) {
    if (index < 0 || index >= self.mesh_material_count()) return INDEX_OUT_OF_RANGE~;
    set_mesh_material_raw(self, material, index);
}
