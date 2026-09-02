module b3;

<*
 The contact between this sphere and another placed by transform_b_to_a, in this sphere's frame,
 the normal pointing from this sphere towards the other.

 A point arrives while the two touch or overlap, so its separation is never positive — exactly
 zero where the surfaces meet, negative once they overlap. Coincident centres report minus the
 summed radii under a +y normal, box3d's fallback direction rather than a measured one.
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

 A point arrives while the two touch or overlap, so its separation is never positive — zero where
 the surfaces meet. A capsule whose centres coincide is collided as the sphere they name, and a
 sphere on the capsule's axis reports a +y normal, box3d's fallback where there is none to measure.
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

 A four-slop speculative window, so a point can arrive with a positive separation for two shapes
 not yet touching. A sphere centre inside the hull is answered from the nearest face — box3d takes
 the greatest separation, which inside is the least negative.
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
 the normal pointing towards the other. Two points arrive only when they are near parallel, overlap
 along their axes and touch; anything else gives one, and the four-slop window reaches only the
 one-point branch. Nothing at all is reported when either capsule is shorter than a linear slop, or
 the axes pass within a hundredth of one of each other — coaxial as well as crossing.
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
