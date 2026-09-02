module b3;

<*
 The contact between this sphere and another placed by transform_b_to_a, in this sphere's frame,
 the normal pointing from this sphere towards the other.

 A point arrives only while the two actually overlap, so the separation it carries is always
 negative. Coincident centres report minus the summed radii under a +y normal, which is box3d's
 fallback direction rather than a measured one.
*>
fn CollisionManifold Sphere.collide_sphere(&self, Sphere other, Transform transform_b_to_a) {
    CollisionManifold contact;
    LocalManifold manifold = { .points = &contact.points[0] };
    collide_spheres_raw(
      manifold: &manifold,
      capacity: MAX_MANIFOLD_POINTS,
      sphere_a: self,
      sphere_b: &other,
      transform_b_to_a: transform_b_to_a,
    );
    contact.normal = manifold.normal;
    contact.point_count = manifold.point_count;
    return contact;
}

<*
 The contact between this capsule and a sphere placed by transform_b_to_a, in the capsule's frame,
 the normal pointing from the capsule towards the sphere.

 A point arrives only while the two actually overlap, so the separation it carries is always
 negative. A capsule whose two centres coincide is collided as the sphere they name, and a sphere
 centred on the capsule's axis reports a +y normal, box3d's fallback where there is none to measure.
*>
fn CollisionManifold Capsule.collide_sphere(&self, Sphere sphere, Transform transform_b_to_a) {
    CollisionManifold contact;
    LocalManifold manifold = { .points = &contact.points[0] };
    collide_capsule_and_sphere_raw(
      manifold: &manifold,
      capacity: MAX_MANIFOLD_POINTS,
      capsule_a: self,
      sphere_b: &sphere,
      transform_b_to_a: transform_b_to_a,
    );
    contact.normal = manifold.normal;
    contact.point_count = manifold.point_count;
    return contact;
}

<*
 The contact between this hull and a sphere placed by transform_b_to_a, in the hull's frame, the
 normal pointing from the hull towards the sphere.

 This pair carries a speculative window of four linear slops, so a point can arrive with a
 positive separation for two shapes that are not yet touching. A sphere centre inside the hull is
 answered from the face plane it lies deepest against instead.
*>
fn CollisionManifold HullData.collide_sphere(&self, Sphere sphere, Transform transform_b_to_a) {
    CollisionManifold contact;
    LocalManifold manifold = { .points = &contact.points[0] };
    SimplexCache warm_start;
    collide_hull_and_sphere_raw(
      manifold: &manifold,
      capacity: MAX_MANIFOLD_POINTS,
      hull_a: self,
      sphere_b: &sphere,
      transform_b_to_a: transform_b_to_a,
      cache: &warm_start,
    );
    contact.normal = manifold.normal;
    contact.point_count = manifold.point_count;
    return contact;
}

<*
 The contact between this capsule and another placed by transform_b_to_a, in this capsule's frame,
 the normal pointing from this capsule towards the other.

 Two nearly parallel capsules give two points and anything else one, and the speculative window of
 four linear slops reaches only the one-point branch. Either capsule shorter than a linear slop,
 or two axes that exactly cross, reports nothing at all however deep the overlap.
*>
fn CollisionManifold Capsule.collide_capsule(&self, Capsule other, Transform transform_b_to_a) {
    CollisionManifold contact;
    LocalManifold manifold = { .points = &contact.points[0] };
    collide_capsules_raw(
      manifold: &manifold,
      capacity: MAX_MANIFOLD_POINTS,
      capsule_a: self,
      capsule_b: &other,
      transform_b_to_a: transform_b_to_a,
    );
    contact.normal = manifold.normal;
    contact.point_count = manifold.point_count;
    return contact;
}

<*
 The contact points this manifold carries — the first point_count of its slots. The slice borrows
 the manifold it was called on, so one taken straight from the call that produced a manifold
 outlives it: store the manifold first.

 @return "the points the pair produced, between none and MAX_MANIFOLD_POINTS of them"
*>
fn LocalManifoldPoint[] CollisionManifold.contact_points(&self) {
    return self.points[:self.point_count];
}
