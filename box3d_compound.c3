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
 Attaches a baked compound to this body. The definition must come from default_shape_def() — see
 create_sphere_shape — and the null guard is the one create_mesh_shape carries for the same
 reason: box3d reads a version field through the pointer before it tests anything.

 box3d takes a baked compound only on a static body and hands back a null identifier for a
 kinematic or a dynamic one, so INVALID_HANDLE here usually means the body was the wrong type
 rather than that anything is wrong with the compound. A body that has to move carries its
 children as separate shapes on the body instead.

 The shape borrows the compound rather than copying it, exactly as a mesh shape borrows its mesh
 — see create_mesh_shape for what that costs and for what destroying the geometry first does.

 What the shape does copy is the compound's materials, not the definition's: mesh_material_count()
 answers with one per distinct child material and mesh_surface_material() indexes them in the
 order the children were baked, which is the order CompoundDef lists its kinds. Two children
 sharing a material share one entry, and the material index a cast reports indexes the same array.

 @param [&in] def : "a definition obtained from default_shape_def()"
 @param compound : "the compound, which must outlive the shape; null is refused rather than dereferenced"
 @return "the shape, INVALID_DEFINITION if the definition did not come from default_shape_def() or the compound is null, or INVALID_HANDLE if box3d returns a null identifier, which a body that is not static always does"
*>
fn ShapeId? BodyId.create_baked_compound_shape(self, ShapeDef* def, CompoundData* compound) {
    if (def.internal_value != default_shape_def().internal_value) return INVALID_DEFINITION~;
    if (compound == null) return INVALID_DEFINITION~;
    return create_baked_compound_shape_raw(self, def, compound).checked();
}
