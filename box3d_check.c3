module b3;

faultdef
    WORLD_LIMIT_REACHED,
    INVALID_DEFINITION,
    INVALID_HANDLE,
    DOUBLE_PRECISION_LIBRARY,
    WRONG_SHAPE_TYPE,
    WRONG_JOINT_TYPE,
    DEGENERATE_GEOMETRY,
    INDEX_OUT_OF_RANGE,
    CHILD_LIMIT_REACHED,
    NO_HIT;

macro bool WorldId.is_null(self) => self.index1 == 0;
macro bool BodyId.is_null(self) => self.index1 == 0;
macro bool ShapeId.is_null(self) => self.index1 == 0;
macro bool JointId.is_null(self) => self.index1 == 0;
macro bool ContactId.is_null(self) => self.index1 == 0;

<*
 True when the package was compiled with the BOX3D_CHECKED feature. The feature gates nothing on
 its own — validity checking is opt-in per call site through the .checked() macros — so this
 reports the build's intent for a caller that wants to branch on it.
*>
macro bool checked_access() {
    $if $feat(BOX3D_CHECKED):
        return true;
    $else
        return false;
    $endif
}

<*
 @return "the world, or INVALID_HANDLE when box3d returned a null identifier"
*>
macro WorldId? WorldId.checked(self) {
    if (self.is_null()) return INVALID_HANDLE~;
    return self;
}

<*
 @return "the body, or INVALID_HANDLE when box3d returned a null identifier"
*>
macro BodyId? BodyId.checked(self) {
    if (self.is_null()) return INVALID_HANDLE~;
    return self;
}

<*
 @return "the shape, or INVALID_HANDLE when box3d returned a null identifier"
*>
macro ShapeId? ShapeId.checked(self) {
    if (self.is_null()) return INVALID_HANDLE~;
    return self;
}

<*
 @return "the joint, or INVALID_HANDLE when box3d returned a null identifier"
*>
macro JointId? JointId.checked(self) {
    if (self.is_null()) return INVALID_HANDLE~;
    return self;
}

<*
 @return "the contact, or INVALID_HANDLE when box3d returned a null identifier"
*>
macro ContactId? ContactId.checked(self) {
    if (self.is_null()) return INVALID_HANDLE~;
    return self;
}

<*
 A null pointer here is not a bad handle but a refusal to build: box3d returns one when the
 vertices and indices it was given hold no triangle with area. This is the one member of the
 family that cannot be a method, because C3 traps a method call on a null receiver before the
 body can test it.

 @return "the mesh, or DEGENERATE_GEOMETRY when box3d returned a null pointer"
*>
macro MeshData*? checked_mesh(MeshData* mesh) {
    if (mesh == null) return DEGENERATE_GEOMETRY~;
    return mesh;
}
