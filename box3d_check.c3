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
    NOT_A_PROXY,
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
 A null pointer here is not a bad handle but a refusal to build: box3d returns one when the given
 vertices and indices hold no triangle with area. A macro rather than a method because C3 traps a
 method call on a null receiver before the body can test it.

 @return "the mesh, or DEGENERATE_GEOMETRY when box3d returned a null pointer"
*>
macro MeshData*? checked_mesh(MeshData* mesh) {
    if (mesh == null) return DEGENERATE_GEOMETRY~;
    return mesh;
}

<*
 A null compound is box3d refusing to bake one, which it does for a single reason: a definition
 naming as many children as its own ceiling allows. A macro rather than a method, as checked_mesh()
 is.

 @return "the compound, or CHILD_LIMIT_REACHED when box3d returned a null pointer"
*>
macro CompoundData*? checked_compound(CompoundData* compound) {
    if (compound == null) return CHILD_LIMIT_REACHED~;
    return compound;
}

<*
 The range and leaf checks box3d compiles out of every call that indexes the node array on an id
 the caller supplies. The leaf bit is exact for a live proxy but says nothing about which one: an
 id whose slot has since been handed to a new proxy passes.

 @return "INDEX_OUT_OF_RANGE when the id is outside 0 to node_capacity - 1, or NOT_A_PROXY when no live proxy holds that node"
*>
fn void? check_proxy(DynamicTree* tree, int proxy_id) @private {
    if (proxy_id < 0 || proxy_id >= tree.node_capacity) return INDEX_OUT_OF_RANGE~;
    if ((tree.nodes[proxy_id].flags & TREE_NODE_LEAF) == 0) return NOT_A_PROXY~;
}
