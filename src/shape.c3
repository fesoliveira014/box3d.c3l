module b3;

<*
 Attaches a sphere to this body.

 @param [&in] def : `a definition obtained from default_shape_def()`
 @param [&in] sphere : `the sphere in body-local space; box3d clones it`
 @return `the shape`
 @return? check_shape_def!
 @return? INVALID_DEFINITION : `sphere is null`
 @return? INVALID_HANDLE : `box3d returned a null identifier`
*>
fn ShapeId? BodyId.create_sphere_shape(self, ShapeDef* def, Sphere* sphere) {
    check_shape_def(def)!;
    if (sphere == null) return INVALID_DEFINITION~;
    return create_sphere_shape_raw(self, def, sphere).checked();
}

<*
 Attaches a capsule to this body. box3d substitutes a sphere when the two centres fall within its
 linear slop of each other, so the returned shape's type() may be SPHERE rather than CAPSULE.

 @param [&in] def : `a definition obtained from default_shape_def()`
 @param [&in] capsule : `the capsule in body-local space; box3d clones it`
 @return `the shape`
 @return? check_shape_def!
 @return? INVALID_DEFINITION : `capsule is null`
 @return? INVALID_HANDLE : `box3d returned a null identifier`
*>
fn ShapeId? BodyId.create_capsule_shape(self, ShapeDef* def, Capsule* capsule) {
    check_shape_def(def)!;
    if (capsule == null) return INVALID_DEFINITION~;
    return create_capsule_shape_raw(self, def, capsule).checked();
}

<*
 Attaches a convex hull to this body. box3d clones the hull, so a stack BoxHull is enough and
 nothing needs to outlive the call.

 @param [&in] def : `a definition obtained from default_shape_def()`
 @param [&in] hull : `the hull in body-local space, usually the base of a BoxHull`
 @return `the shape`
 @return? check_shape_def!
 @return? INVALID_DEFINITION : `hull is null`
 @return? INVALID_HANDLE : `box3d returned a null identifier`
*>
fn ShapeId? BodyId.create_hull_shape(self, ShapeDef* def, HullData* hull) {
    check_shape_def(def)!;
    if (hull == null) return INVALID_DEFINITION~;
    return create_hull_shape_raw(self, def, hull).checked();
}

<*
 Attaches a convex hull to this body with a scale and a local transform baked into it. Scale
 applies before the transform and may be non-uniform or mirrored.

 @param [&in] def : `a definition obtained from default_shape_def()`
 @param [&in] hull : `the hull to bake, usually the base of a BoxHull`
 @return `the shape`
 @return? check_shape_def!
 @return? INVALID_DEFINITION : `hull is null`
 @return? INVALID_HANDLE : `box3d returned a null identifier`
*>
fn ShapeId? BodyId.create_transformed_hull_shape(
  self,
  ShapeDef* def,
  HullData* hull,
  Transform transform,
  Vec3 scale,
) {
    check_shape_def(def)!;
    if (hull == null) return INVALID_DEFINITION~;
    return create_transformed_hull_shape_raw(
      body_id: self,
      def: def,
      hull: hull,
      transform: transform,
      scale: scale,
    ).checked();
}

<*
 @return `the sphere geometry`
 @return? WRONG_SHAPE_TYPE : `this shape is not a sphere`
*>
fn Sphere? ShapeId.sphere(self) {
    if (self.type() != ShapeType.SPHERE) return WRONG_SHAPE_TYPE~;
    return sphere_raw(self);
}

<*
 @return `the capsule geometry`
 @return? WRONG_SHAPE_TYPE : `this shape is not a capsule`
*>
fn Capsule? ShapeId.capsule(self) {
    if (self.type() != ShapeType.CAPSULE) return WRONG_SHAPE_TYPE~;
    return capsule_raw(self);
}

<*
 The hull belongs to the world, which reference-counts a clone of whatever built the shape, and
 stays valid until the shape is destroyed or given other geometry. Calling destroy() on it takes
 the world's copy out from under every shape sharing it, and the next step or world teardown reads
 freed memory.

 @return `the hull`
 @return? WRONG_SHAPE_TYPE : `this shape is not a hull`
*>
fn HullData*? ShapeId.hull(self) {
    if (self.type() != ShapeType.HULL) return WRONG_SHAPE_TYPE~;
    return hull_raw(self);
}

<*
 The mesh belongs to the caller rather than to the world: data is the very pointer
 BodyId.create_mesh_shape() or set_mesh() was handed, and it must still outlive this shape.

 @return `the mesh and the scale it is read through`
 @return? WRONG_SHAPE_TYPE : `this shape is not a mesh`
*>
fn Mesh? ShapeId.mesh(self) {
    if (self.type() != ShapeType.MESH) return WRONG_SHAPE_TYPE~;
    return mesh_raw(self);
}

<*
 The height field belongs to the caller rather than to the world — see mesh() — and it too must
 outlive this shape.

 @return `the height field`
 @return? WRONG_SHAPE_TYPE : `this shape is not a height field`
*>
fn HeightFieldData*? ShapeId.height_field(self) {
    if (self.type() != ShapeType.HEIGHT_FIELD) return WRONG_SHAPE_TYPE~;
    return height_field_raw(self);
}

<*
 Gives this shape a mesh, replacing whatever geometry it had and making its type MESH — there is
 no type guard, unlike on mesh(), because changing the type is what the call is for.

 The shape borrows the new mesh as BodyId.create_mesh_shape() does, and box3d drops the old
 pointer without freeing it, so a caller replacing a mesh is left holding two and owes destroy()
 on both. The body's mass is not recomputed until BodyId.apply_mass_from_shapes(), and a mesh
 contributes none. Silently does nothing while the world is locked, mid WorldId.step.

 @param mesh : `the mesh, which must outlive the shape`
 @param scale : `how far the mesh's vertices are stretched along each axis, non-uniform or
   mirrored; a zero component is pushed away from zero rather than refused`
 @return? INVALID_DEFINITION : `mesh is null, which is what create_mesh_shape answers there too`
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

 @return `the material`
 @return? INDEX_OUT_OF_RANGE : `index is negative or not below mesh_material_count()`
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

 @return? INDEX_OUT_OF_RANGE : `index is negative or not below mesh_material_count()`
*>
fn void? ShapeId.set_mesh_material(self, SurfaceMaterial material, int index) {
    if (index < 0 || index >= self.mesh_material_count()) return INDEX_OUT_OF_RANGE~;
    set_mesh_material_raw(self, material, index);
}

<*
 The definition guard every shape constructor runs. This release build compiles box3d's cookie
 assertion out, so an unmarked definition is accepted silently and yields a shape of zero density
 that contributes no mass. The cookie is the only compiled-out assertion this restores — density
 and the material coefficients still go unchecked.

 @param [&in] def : `a definition obtained from default_shape_def()`
 @return? INVALID_DEFINITION : `def is null, or did not come from default_shape_def()`
*>
fn void? check_shape_def(ShapeDef* def) @private {
    if (def == null) return INVALID_DEFINITION~;
    if (def.internal_value != default_shape_def().internal_value) return INVALID_DEFINITION~;
}
