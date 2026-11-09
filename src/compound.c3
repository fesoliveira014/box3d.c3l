module b3;

<*
 Bakes the definition's children — spheres, capsules, hulls and meshes — into one compound shape,
 which the caller owns and must destroy_compound(), as with MeshData. box3d clones every child.
 It refuses only MAX_CHILD_SHAPES children or more; the other guards restore checks it omits or
 asserts in a build that compiles them out. It also asserts at most 65535 distinct materials,
 unguarded here: past that, child material indices wrap silently.

 @param [&in] def : `the children to bake`
 @return `the compound`
 @return? CHILD_LIMIT_REACHED : `the definition names MAX_CHILD_SHAPES children or more`
 @return? INVALID_DEFINITION : `def is null, a count is negative, no child is named, a named array
   is missing, a hull or mesh child names no geometry, or a mesh child names no materials`
*>
fn CompoundData*? create_compound(CompoundDef* def) {
    if (def == null) return INVALID_DEFINITION~;
    if (def.capsule_count < 0 || def.hull_count < 0) return INVALID_DEFINITION~;
    if (def.mesh_count < 0 || def.sphere_count < 0) return INVALID_DEFINITION~;

    long child_count = (long)def.capsule_count + (long)def.hull_count + (long)def.mesh_count + (long)def.sphere_count;
    if (child_count == 0) return INVALID_DEFINITION~;
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
        if (mesh.material_count < 1) return INVALID_DEFINITION~;
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
 filled the definition in. box3d bounds the index with an assertion the release build compiles
 out, and both directions run past the array: an index at or above the count reaches past the
 spheres, a negative one reaches memory before them, and neither reports anything.

 @return `the child`
 @return? INDEX_OUT_OF_RANGE : `the index is outside 0 to child_count() - 1`
*>
fn ChildShape? CompoundData.child(&self, int index) {
    if (index < 0 || index >= self.child_count()) return INDEX_OUT_OF_RANGE~;
    return compound_child_raw(self, index);
}

<*
 The capsule child at index, counting capsules alone. box3d's own bound on the index is an
 assertion the release build compiles out — see child().

 @return `the capsule child`
 @return? INDEX_OUT_OF_RANGE : `the index is outside 0 to capsule_count - 1`
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

 @return `the hull child`
 @return? INDEX_OUT_OF_RANGE : `the index is outside 0 to hull_count - 1`
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

 @return `the mesh child`
 @return? INDEX_OUT_OF_RANGE : `the index is outside 0 to mesh_count - 1`
*>
fn CompoundMesh? CompoundData.mesh(&self, int index) {
    if (index < 0 || index >= self.mesh_count) return INDEX_OUT_OF_RANGE~;
    return compound_mesh_raw(self, index);
}

<*
 The sphere child at index, counting spheres alone. box3d's own bound on the index is an assertion
 the release build compiles out — see child().

 @return `the sphere child`
 @return? INDEX_OUT_OF_RANGE : `the index is outside 0 to sphere_count - 1`
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
 lasts as long as the compound. C returns this array const: treat it as a view.
*>
fn SurfaceMaterial[] CompoundData.materials(&self) {
    return compound_materials_raw(self)[:self.material_count];
}

<*
 The mesh child's material slots, which a triangle's material index selects among. The slice
 borrows the compound's storage and lasts as long as the compound.
*>
fn ushort[] CompoundMesh.material_slots(self) {
    return self.material_indices[:self.material_count];
}

<*
 The child's material slots: one for a convex child, the mesh's slots for a mesh child. The slice
 borrows the compound's storage and lasts as long as the compound.
*>
fn ushort[] ChildShape.material_slots(self) {
    return self.material_indices[:self.material_count];
}

<*
 Reads serialized compound bytes as a compound in place: the result is bytes.ptr, so the bytes stay
 the caller's, must outlive every use, and never go to destroy_compound(). box3d checks the
 version, the length and the node array, and no other offset, so the bytes must be a compound's
 byte_count bytes as this build wrote them. C returns the compound const: treat it as a view.

 @return `the compound`
 @return? NOT_A_COMPOUND : `the bytes are too short, carry another version or length, or hold no
   node array that fits`
*>
fn CompoundData*? validate_compound(char[] bytes) {
    if (bytes.len > int::max) return NOT_A_COMPOUND~;
    return checked_geometry(validate_compound_raw(bytes.ptr, (int)bytes.len), NOT_A_COMPOUND);
}

<*
 Attaches a baked compound to this body, borrowing it as create_mesh_shape borrows its mesh.

 box3d takes a baked compound only on a static body and hands back a null identifier for a
 kinematic or a dynamic one, so INVALID_HANDLE here usually means the body was the wrong type.
 What the shape copies is the compound's materials, not the definition's, so mesh_material_count()
 and mesh_surface_material() answer with CompoundData.materials() over again.

 @param [&in] def : `a definition obtained from default_shape_def()`
 @param compound : `the compound, which must outlive the shape`
 @return `the shape`
 @return? check_shape_def!
 @return? INVALID_DEFINITION : `compound is null`
 @return? INVALID_HANDLE : `box3d returned a null identifier, which a body that is not static
   always does`
*>
fn ShapeId? create_baked_compound_shape(BodyId body, ShapeDef* def, CompoundData* compound) {
    check_shape_def(def)!;
    if (compound == null) return INVALID_DEFINITION~;
    return create_baked_compound_shape_raw(body, def, compound).checked();
}
