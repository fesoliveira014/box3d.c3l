module b3;

<*
 Bakes the definition's children — spheres, capsules, hulls and meshes — into one compound shape.

 The caller owns the compound and must destroy() it, as with MeshData; nothing in the definition
 has to outlive the call, because box3d clones every child. An empty definition builds a childless
 compound, owned like any other. box3d itself refuses only a definition naming MAX_CHILD_SHAPES
 children or more, and refuses before allocating; every other guard below restores a check it does
 not make at all: a null array segfaults, and a negative count hangs the allocator.

 @param [&in] def : "the children to bake"
 @return "the compound, CHILD_LIMIT_REACHED when the definition names MAX_CHILD_SHAPES children or more or box3d refuses one for its own ceiling, or INVALID_DEFINITION when a count is negative, a named array is missing, a hull or mesh child names no geometry, or a mesh child's material count is outside 1 to MAX_COMPOUND_MESH_MATERIALS"
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

    return checked_geometry(create_compound_raw(def), CHILD_LIMIT_REACHED);
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

 The order is box3d's baking order — capsules, hulls, meshes, spheres — not the order the caller
 filled the definition in. box3d bounds the index with an assertion the release build compiles out,
 and both directions run past the array: an index at or above the count reaches past the spheres, a
 negative one reaches memory before them, and neither reports anything.

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

 The hull it reports points inside the compound and belongs to it; destroying it separately frees
 a pointer that was never separately allocated — see CompoundHull.

 @return "the hull child, or INDEX_OUT_OF_RANGE when the index is outside 0 to hull_count - 1"
*>
fn CompoundHull? CompoundData.hull(&self, int index) {
    if (index < 0 || index >= self.hull_count) return INDEX_OUT_OF_RANGE~;
    return compound_hull_raw(self, index);
}

<*
 The mesh child at index, counting meshes alone. box3d's own bound on the index is an assertion
 the release build compiles out — see child().

 The mesh it reports is the compound's clone, not the mesh create_compound() was given, and
 belongs to the compound; destroying it aborts the process — see CompoundMesh.

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
 baked; a child's material index and the one a cast reports both name into this array.

 Distinct is byte-for-byte, so two children agreeing on friction and restitution but differing in
 user_material_id or custom_color get an entry each. The slice borrows the compound's storage and
 lasts as long as the compound; a childless compound gives an empty slice over a non-null pointer
 past the allocation. C returns this array const: treat it as a view, and read the count for empty.
*>
fn SurfaceMaterial[] CompoundData.materials(&self) {
    return compound_materials_raw(self)[:self.material_count];
}

<*
 The bounds of the compound placed by this transform: the box its tree root holds, which is the
 union of every child's, widened under rotation exactly as HullData.compute_aabb() is.

 A compound with no children carries the root -1 and box3d indexes its node array with it, a read
 landing back inside the compound's own header that answers a zero-extent box at the transform's
 position — a plausible empty box rather than an obvious wrong one, which is why it is refused.

 @return "the bounds, or DEGENERATE_GEOMETRY when the compound holds no children"
*>
fn Aabb? CompoundData.compute_aabb(&self, Transform transform) {
    if (self.tree.root < 0) return DEGENERATE_GEOMETRY~;
    return compute_compound_aabb_raw(self, transform);
}

<*
 The compound's own bytes, ready to be written out, with the one address it holds nulled so they
 carry none. They are neither a copy nor a second allocation — destroy() is what frees them — and
 the nulling is not cosmetic: every query, cast and overlap against this compound walks that
 pointer without testing it, so each one crashes until compound_from_bytes() is called over these
 very bytes. Called over a copy it repairs the copy and leaves this compound broken.
*>
fn char[] CompoundData.to_bytes(&self) {
    return compound_to_bytes_raw(self)[:self.byte_count];
}

<*
 Points a compound's tree back into the caller's own bytes and hands them back read as one. It
 allocates nothing and takes nothing over: the bytes stay the caller's, so destroy() is right
 only for a compound box3d allocated. box3d reads the header through the pointer before it checks
 any length, which is what the first guard is for.

 @return "the compound, BUFFER_TOO_SMALL when the slice is shorter than a compound header, or NOT_A_COMPOUND when it carries another version, another length, or no tree"
*>
fn CompoundData*? compound_from_bytes(char[] bytes) {
    if (bytes.len < CompoundData::size) return BUFFER_TOO_SMALL~;
    return checked_geometry(compound_from_bytes_raw(bytes.ptr, (int)bytes.len), NOT_A_COMPOUND);
}

<*
 Attaches a baked compound to this body, under the definition and null guards create_mesh_shape
 describes and borrowing the compound exactly as that call borrows its mesh.

 box3d takes a baked compound only on a static body and hands back a null identifier for a
 kinematic or a dynamic one, so INVALID_HANDLE here usually means the body was the wrong type. What
 the shape copies is the compound's materials, not the definition's, so mesh_material_count() and
 mesh_surface_material() answer with CompoundData.materials() over again.

 @param [&in] def : "a definition obtained from default_shape_def()"
 @param compound : "the compound, which must outlive the shape; null is refused rather than dereferenced"
 @return "the shape, INVALID_DEFINITION if the definition did not come from default_shape_def() or the compound is null, or INVALID_HANDLE if box3d returns a null identifier, which a body that is not static always does"
*>
fn ShapeId? BodyId.create_baked_compound_shape(self, ShapeDef* def, CompoundData* compound) {
    if (def.internal_value != default_shape_def().internal_value) return INVALID_DEFINITION~;
    if (compound == null) return INVALID_DEFINITION~;
    return create_baked_compound_shape_raw(self, def, compound).checked();
}
