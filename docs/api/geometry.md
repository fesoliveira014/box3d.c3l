# Geometry

[Documentation](../index.md) › [Public API](index.md) › Geometry

Standalone geometry that needs no world: contact manifolds between pairs of shapes, distance
between convex proxies, shape casts and time of impact. Sources: `src/collide.c3i`,
`src/distance.c3i` and their wrappers. Every call here is a pure function of its arguments and
allocates nothing.

Related standalone calls live elsewhere: ray casts, shape casts and overlaps against one
primitive (`b3::ray_cast_sphere`, `b3::shape_cast_hull`, `b3::overlap_mesh`, ...) are on
[Queries and casts](queries_and_casts.md), together with `ShapeProxy`, `CastOutput` and
`MAX_MANIFOLD_POINTS`. Segment and line distance (`b3::segment_distance`,
`b3::point_to_segment_distance`, `b3::line_distance`) are on [Library and
math](library_and_math.md).

## Contact manifolds

```c3
b3::Sphere ball = { .center = { 0, 0, 0 }, .radius = 0.5f };
b3::Sphere other = { .center = { 0, 0, 0 }, .radius = 0.5f };
b3::Transform b_in_a = { .p = { 0.9f, 0, 0 }, .q = { .v = { 0, 0, 0, 1 } } };

b3::CollisionManifold contact = b3::collide_spheres(&ball, other, b_in_a);
foreach (point : contact.contact_points()) {
    float depth = -point.separation;
}
```

Each `collide_*` function takes shape A, shape B, and the transform of B in A's frame, and
returns the contact in A's frame with the normal pointing from A towards B.

| Function | Pair | Result |
|---|---|---|
| `b3::collide_spheres` | sphere, sphere | `CollisionManifold` |
| `b3::collide_capsule_and_sphere` | capsule, sphere | `CollisionManifold` |
| `b3::collide_capsules` | capsule, capsule | `CollisionManifold` |
| `b3::collide_hull_and_sphere` | hull, sphere | `CollisionManifold` |
| `b3::collide_hull_and_capsule` | hull, capsule | `CollisionManifold` |
| `b3::collide_hulls` | hull, hull | `CollisionManifold?` |
| `b3::collide_triangle_and_sphere` | triangle, sphere | `CollisionManifold?` |
| `b3::collide_triangle_and_capsule` | triangle, capsule | `CollisionManifold?` |
| `b3::collide_triangle_and_hull` | triangle, hull | `CollisionManifoldView?` over a caller buffer |

A `CollisionManifold` owns its points: `normal`, `point_count` and up to `MAX_MANIFOLD_POINTS`
`LocalManifoldPoint` entries. `contact_points()` returns the valid prefix as a slice that borrows
the manifold, so store the manifold in a variable before taking the slice. A pair that does not
touch returns `point_count` 0.

A `LocalManifoldPoint` holds the `point`, its `separation` (zero where the surfaces meet,
negative for overlap, positive for a speculative point not yet reached) and a `FeaturePair`
naming the two features the point lies between. `triangle_index` is not written by these calls.

```c3
b3::BoxHull a = b3::make_cube_hull(0.5f);
b3::BoxHull b = b3::make_cube_hull(0.5f);
b3::Transform b_in_a = { .p = { 0, 0.95f, 0 }, .q = { .v = { 0, 0, 0, 1 } } };

b3::CollisionManifold contact = b3::collide_hulls(&a.base, &b.base, b_in_a)!;
b3::Vec3 normal = contact.normal;
int count = contact.point_count;
```

Per-pair behaviour:

- The sphere-sphere, capsule-sphere and hull pairs, the triangle pairs (triangle-hull only with
  `enable_speculative`) and the one-point branch of capsule-capsule report speculative points out
  to four linear slops.
- Coincident sphere centres, or a sphere on a capsule's axis, report a +y normal: box3d's
  fallback where no direction can be measured.
- `collide_capsules` reports nothing when either capsule is shorter than a linear slop, or when
  the axes pass within a hundredth of a slop of each other.
- The hull-capsule and triangle-capsule pairs clip the capsule's segment onto a face and can
  return a second point far from contact.
- `collide_hulls` returns `INVALID_DEFINITION` when `other` is null. box3d reduces the clipped
  polygon to `MAX_MANIFOLD_POINTS`.

### Triangles

```c3
b3::Triangle tri = { { -2, 0, -2 }, { 0, 0, 2 }, { 2, 0, -2 } };
b3::BoxHull box = b3::make_offset_box_hull(
  hx: 0.5f,
  hy: 0.5f,
  hz: 0.5f,
  offset: { 0, 0.49f, 0 },
);
b3::LocalManifoldPoint[b3::MAX_MANIFOLD_POINTS] buffer;

b3::CollisionManifoldView view = b3::collide_triangle_and_hull(
  triangle: &tri,
  hull: &box.base,
  enable_speculative: true,
  buffer: buffer[..],
)!;
usz count = view.points.len;
```

A `Triangle` is three `Vec3` in the frame of the shape it is collided with; the triangle pairs
take no transform. Its face normal is `(v1 - v0) × (v2 - v0)`, and each pair culls a shape behind
the face. The point tested differs: the sphere's centre, the capsule's segment midpoint, or the
hull's centroid. Every triangle pair returns `DEGENERATE_GEOMETRY` for a triangle with no area.

`collide_triangle_and_hull` writes its points into the caller's `buffer` and returns a
`CollisionManifoldView` whose `points` slice borrows it. box3d truncates the clipped polygon to the
buffer instead of reducing it, so a count equal to the buffer length may have dropped points. A
buffer shorter than `MAX_MANIFOLD_POINTS` returns `BUFFER_TOO_SMALL`; a null hull
`INVALID_DEFINITION`. `enable_speculative` controls the four-slop speculative points.

### 2D hulls

```c3
b3::Point2D[6] points = {
    { .p = { 0, 0 }, .original_index = 0 },
    { .p = { 2, 0 }, .original_index = 1 },
    { .p = { 2, 2 }, .original_index = 2 },
    { .p = { 0, 2 }, .original_index = 3 },
    { .p = { 1, 1 }, .original_index = 4 },
    { .p = { 1, -0.1f }, .original_index = 5 },
};
b3::Point2D[12] buffer;

b3::Point2D[] hull = b3::hull_2d(points[..], buffer[..])!;
b3::Point2D[] square = b3::simplify_hull_2d(hull, 4);
```

These are the two steps box3d uses to reduce a contact manifold, exposed for testing. A `Point2D`
carries a 2D point, its contact's `separation` and its `original_index`.

`b3::hull_2d` writes the convex hull of `points` into `buffer` and returns the filled prefix:
counter-clockwise from the lowest leftmost point, with points on an edge dropped. It sorts and
welds `points` in place; points within a quarter linear slop become one, keeping the lowest
`original_index`. box3d needs room for twice the points while it builds the hull, so a `buffer`
shorter than that returns `BUFFER_TOO_SMALL`. The two slices must not overlap.

`b3::simplify_hull_2d` rewrites a counter-clockwise hull in place and returns its prefix. It drops
the vertex spanning the least area until at most `target` remain, and drops a vertex flatter than
an eighth of a square linear slop even below `target`. It stops at three; a `target` under three
acts as three, and a hull of three or fewer comes back unchanged.

## Distance

```c3
b3::Vec3[1] point_a = { { 0, 0, 0 } };
b3::Vec3[2] segment_b = { { -1, 0, 0 }, { 1, 0, 0 } };

b3::DistanceInput input = {
    .proxy_a = { .points = &point_a[0], .count = 1, .radius = 0.5f },
    .proxy_b = { .points = &segment_b[0], .count = 2, .radius = 0.25f },
    .transform = { .p = { 0, 3, 0 }, .q = { .v = { 0, 0, 0, 1 } } },
    .use_radii = true,
};
b3::DistanceOutput result = b3::shape_distance(input)!;
float gap = result.distance;
```

`b3::shape_distance` runs GJK between two convex `ShapeProxy` values, each with points in its own
shape's frame, `transform` placing B in A's frame. The `DistanceOutput` is in A's frame: closest
points `point_a` and `point_b`, `normal` from A to B, and `distance` clamped at zero. Above,
`distance` is 2.25: 3 between the cores less both radii.

- `use_radii` subtracts both radii and moves the witness points out to each proxy's surface.
  Without it the answer is between the cores.
- `normal` is exactly zero when the cores overlap. Test for the zero normal, not for
  `distance == 0`: when only the radii drove the distance to zero the normal is real and the two
  witness points have crossed.
- Every call starts cold. `simplex_count` is always 0; `iterations` counts on overlap too.
- Each proxy must name a non-null array of 1 to `MAX_SHAPE_CAST_POINTS` points, or the call
  returns `INVALID_DEFINITION`. The points are read during the call only.

## Shape cast

```c3
b3::Vec3[1] point_a = { { 0, 0, 0 } };
b3::Vec3[1] point_b = { { 0, 0, 0 } };

b3::ShapeCastPairInput input = {
    .proxy_a = { .points = &point_a[0], .count = 1, .radius = 0.5f },
    .proxy_b = { .points = &point_b[0], .count = 1, .radius = 0.25f },
    .transform = { .p = { 0, 4, 0 }, .q = { .v = { 0, 0, 0, 1 } } },
    .translation_b = { 0, -8, 0 },
    .max_fraction = 1.0f,
};
if (catch err = b3::shape_cast(input)) {
    if (err == b3::NO_HIT) return;
    return err~;
}
```

`b3::shape_cast` sweeps proxy B along `translation_b` against a fixed proxy A, in A's frame, and
returns a `CastOutput` with the hit `fraction`, `point` and `normal`. A miss, or a touch past
`max_fraction`, returns `NO_HIT`. `max_fraction` outside 0 to 1 returns `INVALID_DEFINITION`, as
does an invalid proxy.

The cast stops when the cores are `max(slop, total_radius - slop)` apart, with `slop` the linear
slop. The stop is short of the geometric touch below a total radius of one slop, on it at one
slop, and past it above; from two slops up it is a full slop past. Allow that band in any
comparison.

A pair whose cores start overlapped returns fraction 0 with a zero `normal`. `can_encroach` acts
only when the cores start apart but inside the stop distance: more than two slops apart, the pair
closes a further slop; nearer, it advances to half a slop apart, or misses if it is not closing.
Either reports a real fraction and normal. `child_index` and `material_index` read 0.

## Time of impact

```c3
b3::Vec3[1] point = { { 0, 0, 0 } };
b3::Quat level = { .v = { 0, 0, 0, 1 } };

b3::ToiInput input = {
    .proxy_a = { .points = &point[0], .count = 1, .radius = 0.5f },
    .proxy_b = { .points = &point[0], .count = 1, .radius = 0.5f },
    .sweep_a = { .c1 = { 0, 0, 0 }, .c2 = { 0, 0, 0 }, .q1 = level, .q2 = level },
    .sweep_b = { .c1 = { 5, 0, 0 }, .c2 = { -5, 0, 0 }, .q1 = level, .q2 = level },
    .max_fraction = 1.0f,
};
b3::ToiOutput impact = b3::time_of_impact(input)!;
if (impact.state == b3::ToiState.HIT) {
    b3::Transform at_impact = input.sweep_b.transform_at(impact.fraction)!;
}
```

A `Sweep` describes one shape's motion over an interval: the centre of mass from `c1` to `c2`,
the rotation from `q1` to `q2`, and `local_center` placing the shape's origin relative to its
centre of mass. `Sweep.transform_at(t)` returns the shape-origin transform at a fraction of the
interval. The rotation is a normalised lerp, not a slerp. A `t` outside 0 to 1 returns
`INVALID_DEFINITION`.

`b3::time_of_impact` sweeps both proxies and reports the first touch in a `ToiOutput`. Its
`state` is one `ToiState`:

| `ToiState` | Meaning | `fraction`, `point`, `normal` |
|---|---|---|
| `HIT` | the shapes touch inside the interval | the touch; measured point and normal |
| `OVERLAPPED` | the cores overlap at the start | 0; midway witness point, zero normal |
| `SEPARATED` | no touch up to `max_fraction` | `max_fraction`; last witness pair |
| `FAILED` | the solver gave up | the last time it reached |
| `UNKNOWN` | never returned | |

The stop follows the same slop band as `shape_cast`: the example above reports 0.4005 where the
geometric touch is at 0.4. `used_fallback` is always false from a direct call; box3d sets it only
inside its own continuous solver. `distance_iterations`, `push_back_iterations` and
`root_iterations` are diagnostics. An invalid proxy or a `max_fraction` outside 0 to 1 returns
`INVALID_DEFINITION`.

## Contracts

- Manifolds, distances and casts are in shape A's frame, with B placed by the transform given.
- `CollisionManifold.contact_points()` borrows the manifold; keep the manifold alive.
- `collide_triangle_and_hull` needs a buffer of at least `MAX_MANIFOLD_POINTS` and may truncate
  silently at the buffer's length.
- Triangle pairs refuse a zero-area triangle with `DEGENERATE_GEOMETRY`.
- `hull_2d` reorders its input and needs an output buffer of twice the input's length.
- Proxies must hold 1 to `MAX_SHAPE_CAST_POINTS` points through a non-null array; the points are
  borrowed for the call.
- A zero `DistanceOutput.normal` means the cores overlap. Normalising it gives NaN.
- Shape-cast and time-of-impact fractions carry up to one linear slop of error, short or past
  depending on the total radius.
- An initially overlapping shape cast returns fraction 0 with a zero normal unless `can_encroach`
  applies.
