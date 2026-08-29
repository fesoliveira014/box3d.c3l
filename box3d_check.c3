module b3;

faultdef
    WORLD_LIMIT_REACHED,
    INVALID_DEFINITION,
    INVALID_HANDLE,
    DOUBLE_PRECISION_LIBRARY;

macro bool WorldId.is_null(self) => self.index1 == 0;
macro bool BodyId.is_null(self) => self.index1 == 0;
macro bool ShapeId.is_null(self) => self.index1 == 0;
macro bool JointId.is_null(self) => self.index1 == 0;
macro bool ContactId.is_null(self) => self.index1 == 0;

<*
 True when the package was compiled with the BOX3D_CHECKED feature, in which case wrappers
 validate every identifier they are handed.
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
