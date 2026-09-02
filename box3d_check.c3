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
    NO_HIT,
    BUFFER_TOO_SMALL;

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
 A null pointer from one of box3d's geometry builders is not a bad handle but a refusal to build,
 and each builder refuses for its own reason, so the caller names the fault. A macro rather than a
 method because C3 traps a method call on a null receiver before the body can test it.

 @param $fault : "what a null pointer from this builder means"
 @return "the geometry, or $fault when box3d returned a null pointer"
*>
macro checked_geometry(geometry, $fault) {
    if (geometry == null) return $fault~;
    return geometry;
}

<*
 The bounds box3d asserts on a cast or overlap proxy and this build compiles out. A count of none
 is read as one point rather than refused, a count past the ceiling is truncated in silence by the
 compound's sweep and by the mesh and height field overlaps, and a null array is dereferenced.

 @return "INVALID_DEFINITION when the proxy names no array, holds no points, or holds more than MAX_SHAPE_CAST_POINTS"
*>
fn void? check_shape_proxy(ShapeProxy proxy) @private {
    if (proxy.points == null) return INVALID_DEFINITION~;
    if (proxy.count < 1 || proxy.count > MAX_SHAPE_CAST_POINTS) return INVALID_DEFINITION~;
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
