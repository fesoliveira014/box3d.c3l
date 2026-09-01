module b3;

<*
 Bakes the children the definition names into one compound: a tree of spheres, capsules, hulls
 and meshes that a world later populates as a single shape.

 The caller owns the compound: call destroy() on it, and note that byte_count() carries the
 allocation until that happens — ownership works exactly as it does for MeshData, see there.
 Nothing in the definition has to outlive the call, because box3d clones every child it reads,
 the shared hulls and meshes included.

 An empty definition is a request, not a mistake. box3d builds a childless compound for it and
 hands it back, and that compound is owned like any other: it must still be destroyed.

 Only one definition makes box3d itself refuse, and it refuses before allocating anything: one
 naming MAX_CHILD_SHAPES children or more. Every other guard below restores a check box3d does
 not make at all. It reads the four child arrays, and the hull or mesh each child names, without
 testing any of them for null; it writes one index per mesh child material into a fixed array of
 MAX_COMPOUND_MESH_MATERIALS without testing the count against it; and it turns a negative count
 into an unbounded allocation. Each of those was measured as a dead process, not a refusal.

 @param [&in] def : "the children to bake"
 @return "the compound, CHILD_LIMIT_REACHED when the definition names MAX_CHILD_SHAPES children or more, or INVALID_DEFINITION when a count is negative, a named array is missing, a hull or mesh child names no geometry, or a mesh child's material count is outside 1 to MAX_COMPOUND_MESH_MATERIALS"
*>
fn CompoundData*? create_compound(CompoundDef* def) {
    if (def.capsule_count < 0 || def.hull_count < 0) return INVALID_DEFINITION~;
    if (def.mesh_count < 0 || def.sphere_count < 0) return INVALID_DEFINITION~;

    long child_count = (long)def.capsule_count + (long)def.hull_count + (long)def.mesh_count + (long)def.sphere_count;
    if (child_count >= MAX_CHILD_SHAPES) return CHILD_LIMIT_REACHED~;

    if (def.capsule_count > 0 && def.capsules == null) return INVALID_DEFINITION~;
    if (def.hull_count > 0 && def.hulls == null) return INVALID_DEFINITION~;
    if (def.mesh_count > 0 && def.meshes == null) return INVALID_DEFINITION~;
    if (def.sphere_count > 0 && def.spheres == null) return INVALID_DEFINITION~;

    for (int i = 0; i < def.hull_count; i++) {
        if (def.hulls[i].hull == null) return INVALID_DEFINITION~;
    }

    for (int i = 0; i < def.mesh_count; i++) {
        CompoundMeshDef* mesh = &def.meshes[i];
        if (mesh.mesh_data == null || mesh.materials == null) return INVALID_DEFINITION~;
        if (mesh.material_count < 1 || mesh.material_count > MAX_COMPOUND_MESH_MATERIALS) return INVALID_DEFINITION~;
    }

    return create_compound_raw(def);
}

<*
 The number of children the compound holds across all four kinds. This is the bound on the flat
 index child() takes; the four per-kind accessors are bounded by their own count instead.
*>
fn int CompoundData.child_count(&self) {
    return self.capsule_count + self.hull_count + self.mesh_count + self.sphere_count;
}

<*
 The child at a flat index over every kind the compound holds.

 The order is the order box3d baked the children in — capsules, then hulls, then meshes, then
 spheres — and not the order the caller filled the definition in, so a compound whose spheres were
 assigned first still reports its capsules first.

 box3d bounds this index with an assertion the release build compiles out, and both directions run
 past the array behind it: an index at or above the count reaches one element past the spheres,
 and a negative one is made more negative by every kind it walks past and reaches memory before
 them. Neither reports anything; both return a child-shaped read of whatever is there.

 @return "the child, or INDEX_OUT_OF_RANGE when the index is outside 0 to child_count() - 1"
*>
fn ChildShape? CompoundData.child(&self, int index) {
    if (index < 0 || index >= self.child_count()) return INDEX_OUT_OF_RANGE~;
    return compound_child_raw(self, index);
}

<*
 The capsule child at index, counting capsules alone. box3d's own bound on the index is an
 assertion the release build compiles out — see child().

 @return "the capsule child, or INDEX_OUT_OF_RANGE when the index is outside 0 to capsule_count - 1"
*>
fn CompoundCapsule? CompoundData.capsule(&self, int index) {
    if (index < 0 || index >= self.capsule_count) return INDEX_OUT_OF_RANGE~;
    return compound_capsule_raw(self, index);
}

<*
 The hull child at index, counting hulls alone. box3d's own bound on the index is an assertion the
 release build compiles out — see child().

 @return "the hull child, or INDEX_OUT_OF_RANGE when the index is outside 0 to hull_count - 1"
*>
fn CompoundHull? CompoundData.hull(&self, int index) {
    if (index < 0 || index >= self.hull_count) return INDEX_OUT_OF_RANGE~;
    return compound_hull_raw(self, index);
}

<*
 The mesh child at index, counting meshes alone. box3d's own bound on the index is an assertion
 the release build compiles out — see child().

 @return "the mesh child, or INDEX_OUT_OF_RANGE when the index is outside 0 to mesh_count - 1"
*>
fn CompoundMesh? CompoundData.mesh(&self, int index) {
    if (index < 0 || index >= self.mesh_count) return INDEX_OUT_OF_RANGE~;
    return compound_mesh_raw(self, index);
}

<*
 The sphere child at index, counting spheres alone. box3d's own bound on the index is an assertion
 the release build compiles out — see child().

 @return "the sphere child, or INDEX_OUT_OF_RANGE when the index is outside 0 to sphere_count - 1"
*>
fn CompoundSphere? CompoundData.sphere(&self, int index) {
    if (index < 0 || index >= self.sphere_count) return INDEX_OUT_OF_RANGE~;
    return compound_sphere_raw(self, index);
}

<*
 The compound's materials, one entry per distinct child material, in the order the children were
 baked. A child names one of these through its material index, and so does the material index a
 cast reports against a shape built from this compound.

 Distinct is byte-for-byte: box3d compares and hashes whole materials, so two children agreeing on
 friction and restitution but differing in user_material_id or custom_color get an entry each.

 The slice borrows the compound's own storage and lasts exactly as long as the compound. A
 compound with no children has no materials and the slice is empty; box3d hands back a non-null
 pointer even then, addressing the first byte past its allocation, so the count is the only thing
 that reports the emptiness.
*>
fn SurfaceMaterial[] CompoundData.materials(&self) {
    return compound_materials_raw(self)[:self.material_count];
}

<*
 Attaches a baked compound to this body. The definition must come from default_shape_def() — see
 create_sphere_shape — and the null guard is the one create_mesh_shape carries for the same
 reason: box3d reads a version field through the pointer before it tests anything.

 box3d takes a baked compound only on a static body and hands back a null identifier for a
 kinematic or a dynamic one, so INVALID_HANDLE here usually means the body was the wrong type
 rather than that anything is wrong with the compound. A body that has to move carries its
 children as separate shapes on the body instead.

 The shape borrows the compound rather than copying it, exactly as a mesh shape borrows its mesh
 — see create_mesh_shape for what that costs and for what destroying the geometry first does.

 What the shape does copy is the compound's materials, not the definition's, so what
 mesh_material_count() and mesh_surface_material() answer with is CompoundData.materials() over
 again — see there for the order and for what makes two children's materials distinct. The
 material index a cast reports indexes the same array.

 @param [&in] def : "a definition obtained from default_shape_def()"
 @param compound : "the compound, which must outlive the shape; null is refused rather than dereferenced"
 @return "the shape, INVALID_DEFINITION if the definition did not come from default_shape_def() or the compound is null, or INVALID_HANDLE if box3d returns a null identifier, which a body that is not static always does"
*>
fn ShapeId? BodyId.create_baked_compound_shape(self, ShapeDef* def, CompoundData* compound) {
    if (def.internal_value != default_shape_def().internal_value) return INVALID_DEFINITION~;
    if (compound == null) return INVALID_DEFINITION~;
    return create_baked_compound_shape_raw(self, def, compound).checked();
}
