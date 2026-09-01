module b3;

<*
 Builds a flat grid of triangles in the x-z plane, centred on the origin and x_count by z_count
 cells across.

 The caller owns the mesh: call destroy() on it, and note that box3d's byte_count() carries the
 allocation until that happens — see MeshData for why the binding still allocates nothing of
 its own. Every mesh constructor below shares that contract and returns the same fault, and
 none of them touches a world, so none carries a locked-world caveat.

 The refusal is not free of consequence. box3d rejects geometry holding no triangle with area,
 and a hierarchy too deep for its traversal stack, only after cloning the vertices and indices,
 and it frees only part of what it took: the rest stays on byte_count() for the life of the
 process. The refusals that come from counts alone — a grid with no cells, a torus with no
 segments, a definition naming fewer than three vertices or no triangles — are taken before
 box3d allocates anything.

 @param material_count : "how many distinct material indices to spread over the triangles"
 @param identify_edges : "compute the shared-edge adjacency the solver uses to hide inner edges"
 @return "the mesh, or DEGENERATE_GEOMETRY when the arguments describe no triangle with area"
*>
fn MeshData*? create_grid_mesh(int x_count, int z_count, float cell_width, int material_count, bool identify_edges) {
    return checked_mesh(create_grid_mesh_raw(
      x_count: x_count,
      z_count: z_count,
      cell_width: cell_width,
      material_count: material_count,
      identify_edges: identify_edges,
    ));
}

<*
 Builds a grid of triangles in the x-z plane with a sine wave raised over it. Ownership and the
 fault work as they do for create_grid_mesh.

 @return "the mesh, or DEGENERATE_GEOMETRY when the arguments describe no triangle with area"
*>
fn MeshData*? create_wave_mesh(
  int x_count,
  int z_count,
  float cell_width,
  float amplitude,
  float row_frequency,
  float column_frequency,
) {
    return checked_mesh(create_wave_mesh_raw(
      x_count: x_count,
      z_count: z_count,
      cell_width: cell_width,
      amplitude: amplitude,
      row_frequency: row_frequency,
      column_frequency: column_frequency,
    ));
}

<*
 Builds a closed torus around the origin. Ownership and the fault work as they do for
 create_grid_mesh.

 @param radius : "the distance from the centre to the middle of the tube"
 @param thickness : "the radius of the tube itself"
 @return "the mesh, or DEGENERATE_GEOMETRY when either resolution is below 3"
*>
fn MeshData*? create_torus_mesh(int radial_resolution, int tubular_resolution, float radius, float thickness) {
    return checked_mesh(create_torus_mesh_raw(
      radial_resolution: radial_resolution,
      tubular_resolution: tubular_resolution,
      radius: radius,
      thickness: thickness,
    ));
}

<*
 Builds a closed box of twelve triangles wound outwards. Ownership and the fault work as they
 do for create_grid_mesh.

 @param extent : "the box half-widths; a zero component collapses a face and is refused"
 @return "the mesh, or DEGENERATE_GEOMETRY when the extent collapses every triangle"
*>
fn MeshData*? create_box_mesh(Vec3 center, Vec3 extent, bool identify_edges) {
    return checked_mesh(create_box_mesh_raw(center, extent, identify_edges));
}

<*
 Builds a box of twelve triangles wound inwards, so bodies are contained by it rather than
 excluded from it. Ownership and the fault work as they do for create_grid_mesh.

 @return "the mesh, or DEGENERATE_GEOMETRY when the extent collapses every triangle"
*>
fn MeshData*? create_hollow_box_mesh(Vec3 center, Vec3 extent) {
    return checked_mesh(create_hollow_box_mesh_raw(center, extent));
}

<*
 Builds a truncated pyramid standing on the y axis. Ownership and the fault work as they do for
 create_grid_mesh.

 @return "the mesh, or DEGENERATE_GEOMETRY when the height and the two widths collapse it"
*>
fn MeshData*? create_platform_mesh(Vec3 center, float height, float top_width, float bottom_width) {
    return checked_mesh(create_platform_mesh_raw(
      center: center,
      height: height,
      top_width: top_width,
      bottom_width: bottom_width,
    ));
}

<*
 Builds a mesh from an arbitrary triangle soup. Ownership and the fault work as they do for
 create_grid_mesh, and box3d copies the definition's arrays, so nothing has to outlive the call.

 box3d skips triangles whose area is below its tolerance rather than failing on them, and
 refuses the mesh only once nothing is left. It can also report which triangles it skipped;
 that diagnostic is not bound, because the count it goes with lives in the opaque mesh.

 @param [&in] def : "the vertices and indices to build from"
 @return "the mesh, or DEGENERATE_GEOMETRY when the definition names fewer than three vertices, no triangles, or no triangle with area"
*>
fn MeshData*? create_mesh(MeshDef* def) {
    return checked_mesh(create_mesh_raw(def: def, degenerate_triangle_indices: null, degenerate_capacity: 0));
}

<*
 Builds a flat height field of row_count by column_count grid points, sampled at the origin
 corner and running along positive x and z.

 The caller owns the height field: call destroy() on it, and note that byte_count() carries the
 allocation until that happens — see MeshData. None of the three height field constructors
 touches a world, so none carries a locked-world caveat.

 Unlike the mesh constructors, box3d never refuses here. It builds whatever it is asked for,
 including a grid of one row that has no cell and no triangle in it, and its own check that a
 grid holds at least four points is an assertion this release build compiles out. The count
 guard below restores it, and rules out the negative counts that would satisfy it by
 multiplying to a positive product.

 @param scale : "how far apart the grid points sit in x and z, and what the heights are multiplied by in y"
 @param make_holes : "punch every sixteenth cell out of the field"
 @return "the height field, or DEGENERATE_GEOMETRY when either count is below 2"
*>
fn HeightFieldData*? create_height_field_grid(int row_count, int column_count, Vec3 scale, bool make_holes) {
    if (row_count < 2 || column_count < 2) return DEGENERATE_GEOMETRY~;
    return create_grid_raw(
      row_count: row_count,
      column_count: column_count,
      scale: scale,
      make_holes: make_holes,
    );
}

<*
 Builds a height field with two sine waves crossed over it. Ownership, the absent locked-world
 caveat and the count guard all work as they do for create_height_field_grid.

 @return "the height field, or DEGENERATE_GEOMETRY when either count is below 2"
*>
fn HeightFieldData*? create_height_field_wave(
  int row_count,
  int column_count,
  Vec3 scale,
  float row_frequency,
  float column_frequency,
  bool make_holes,
) {
    if (row_count < 2 || column_count < 2) return DEGENERATE_GEOMETRY~;
    return create_wave_raw(
      row_count: row_count,
      column_count: column_count,
      scale: scale,
      row_frequency: row_frequency,
      column_frequency: column_frequency,
      make_holes: make_holes,
    );
}

<*
 Builds a height field from measured heights. Ownership and the absent locked-world caveat work
 as they do for create_height_field_grid, and box3d copies the definition's arrays, so nothing
 has to outlive the call.

 box3d reads count_x * count_z floats through def.heights without testing it for null, and its
 own checks that the grid holds at least four points and that the global range runs upwards are
 assertions this release build compiles out. The three guards below restore all of that.

 @param [&in] def : "the grid to build from"
 @return "the height field, DEGENERATE_GEOMETRY when either count is below 2, or INVALID_DEFINITION when the heights are missing or the global range runs backwards"
*>
fn HeightFieldData*? create_height_field(HeightFieldDef* def) {
    if (def.count_x < 2 || def.count_z < 2) return DEGENERATE_GEOMETRY~;
    if (def.heights == null) return INVALID_DEFINITION~;
    if (def.global_minimum_height > def.global_maximum_height) return INVALID_DEFINITION~;
    return create_height_field_raw(def);
}

<*
 Attaches a mesh to this body. The definition must come from default_shape_def() — see
 create_sphere_shape, and the cookie is the only one of box3d's compiled-out checks restored
 here too.

 The shape borrows the mesh rather than copying it. box3d stores the pointer it is handed and
 reads through it on every step, so the mesh must outlive every shape built from it: destroying
 it first is a use-after-free that nothing reports, and which was measured to both carry a ball
 and drop it through the floor depending on what reused the freed memory. Destroying the world
 takes the shape with it but not the mesh, which stays the caller's to destroy.

 The null guard is the difference between a fault and a segmentation fault: box3d reads a
 version field through the pointer before it tests anything. Scale multiplies the mesh's
 vertices and may be non-uniform or mirrored, and a component of zero is pushed away from zero
 rather than refused.

 @param [&in] def : "a definition obtained from default_shape_def()"
 @param mesh : "the mesh, which must outlive the shape; null is refused rather than dereferenced"
 @param scale : "how far the mesh's vertices are stretched along each axis before collision"
 @return "the shape, INVALID_DEFINITION if the definition did not come from default_shape_def() or the mesh is null, or INVALID_HANDLE if box3d returns a null identifier"
*>
fn ShapeId? BodyId.create_mesh_shape(self, ShapeDef* def, MeshData* mesh, Vec3 scale) {
    if (def.internal_value != default_shape_def().internal_value) return INVALID_DEFINITION~;
    if (mesh == null) return INVALID_DEFINITION~;
    return create_mesh_shape_raw(
      body_id: self,
      def: def,
      mesh: mesh,
      scale: scale,
    ).checked();
}

<*
 Attaches a height field to this body. The definition and null guards, the borrowed geometry and
 the use-after-free that follows destroying it early all work as they do for create_mesh_shape.

 box3d takes a height field only on a static body and hands back a null identifier for a
 kinematic or dynamic one, so INVALID_HANDLE here usually means the body was the wrong type
 rather than that anything is wrong with the field. There is no scale: the one the field was
 built with is the only one it has.

 @param [&in] def : "a definition obtained from default_shape_def()"
 @param height_field : "the field, which must outlive the shape; null is refused rather than dereferenced"
 @return "the shape, INVALID_DEFINITION if the definition did not come from default_shape_def() or the height field is null, or INVALID_HANDLE if box3d returns a null identifier"
*>
fn ShapeId? BodyId.create_height_field_shape(self, ShapeDef* def, HeightFieldData* height_field) {
    if (def.internal_value != default_shape_def().internal_value) return INVALID_DEFINITION~;
    if (height_field == null) return INVALID_DEFINITION~;
    return create_height_field_shape_raw(self, def, height_field).checked();
}
