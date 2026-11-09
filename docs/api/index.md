# Public API

[Documentation](../index.md) › Public API

The public module is `b3`. Debug draw is `b3::draw` and recording and replay is `b3::record`;
`import b3;` reaches both, because C3 imports submodules recursively. Every fault lives in `b3`,
whichever module returns it.

Each domain page walks through setup and use, then lists contracts. The [symbol map](#symbol-map)
at the bottom of this page assigns every public name to one domain. Docstrings in `src/*.c3i` and
`src/*.c3` are the authority for exact signatures and fault lists.

## Conventions

**Names.** The C prefix `b3` is the module. Functions are `snake_case`, types `PascalCase`,
constants and enum values `SCREAMING_SNAKE_CASE`. Getters drop `Get`; setters keep `set_`.

| C | C3 |
|---|---|
| `b3CreateWorld(&def)` | `b3::create_world(&def)!` |
| `b3World_Step(world, dt, 4)` | `b3::world_step(world, dt, 4)` |
| `b3World_GetGravity(world)` | `world.gravity()` |
| `b3Body_SetLinearVelocity(body, v)` | `body.set_linear_velocity(v)` |
| `b3Body_IsAwake(body)` | `body.is_awake()` |
| `b3Body_ComputeAABB(body)` | `body.compute_aabb()` |
| `b3Body_ApplyForce(body, f, p, true)` | `b3::body_apply_force(body, f, p, true)` |
| `b3RecPlayer*` | `record::Player*` |

A C symbol that reads or writes a property (`Get`, `Set`, `Is`, `Are`, `Enable`, `Compute`) is a
method on its type. Every other symbol is a free function taking the receiver first, named from
the C symbol with nothing reordered. See [architecture](../architecture.md#names).

**Identifiers.** `WorldId`, `BodyId`, `ShapeId`, `JointId`, `ContactId`, and the per-type joint
IDs are small value structs, copied freely. `.is_null()` tests for the zero identifier;
`.is_valid()` asks box3d whether the object is still alive; `.checked()` turns a null identifier
into `INVALID_HANDLE`.

**Definitions.** Constructors take a definition by pointer. Start from the matching
`default_*_def()` and change fields. The wrapper refuses a definition that did not come from the
default function with `INVALID_DEFINITION`: box3d's own check is compiled out of the shipped
Release build.

```c3
b3::BodyDef def = b3::default_body_def();
def.type = b3::BodyType.DYNAMIC;
def.position = { 0, 4, 0 };
b3::BodyId body = b3::create_body(world, &def)!;
```

**Optionals.** Every fallible call returns `T?` or `void?` with a fault from the table below.
Propagate with `!`, handle with `if (catch err = ...)`.

```c3
b3::RayResult? hit = b3::world_cast_ray_closest(world, origin, translation, filter);
if (catch err = hit) {
    if (err == b3::NO_HIT) return;
    return err~;
}
```

**Slices.** Accessors that box3d spells as a count plus a pointer return a slice. Event slices
belong to the world and stay valid until the next `world_step`. Accessors that fill a buffer take
a caller slice and return the filled prefix.

**Ownership.** `create_x` and `destroy_x` are free functions. Destroying a world destroys every
body, shape, and joint in it. Geometry that box3d builds for the caller (`MeshData`,
`HeightFieldData`, hulls, compound data) is the caller's to destroy. A mesh or height-field shape
references its data, so the data must outlive the shape; see [shapes](shapes.md#contracts).

**Threading and callbacks.** box3d may call the friction, restitution, pre-solve, and
custom-filter callbacks from a worker thread during a step; they must not touch the world. Query
and cast callbacks run synchronously on the calling thread, and the world must not be modified
from inside them. See [architecture](../architecture.md#threading).

**Checked access.** Wrappers guard the arguments box3d's Release build would otherwise accept
silently and turn into memory corruption: definitions, indices, counts, and the proxies of the
standalone casts. They do not check a stale identifier passed to an ordinary accessor, geometry
freed while a shape still borrows it, or a `ShapeProxy` passed to a world or body query; see
[queries and casts](queries_and_casts.md). Routing an identifier through `.checked()` reports a null one as a fault. The
`BOX3D_CHECKED` feature changes only what `b3::checked_access()` reports.

## Faults

| Fault | Meaning |
|---|---|
| `INVALID_DEFINITION` | A definition, proxy, ray, box, or other argument struct is null, did not come from its `default_*` function, or holds a value box3d asserts on |
| `INVALID_HANDLE` | box3d returned a null identifier, the world is locked (mid step or inside a callback), or a joint narrowed with `as_*()` is no longer alive |
| `WORLD_LIMIT_REACHED` | `MAX_WORLDS` worlds already exist |
| `DOUBLE_PRECISION_LIBRARY` | The linked box3d was built with `BOX3D_DOUBLE_PRECISION` |
| `WRONG_SHAPE_TYPE` | A typed shape accessor was called on a shape of another type |
| `WRONG_JOINT_TYPE` | A `JointId.as_*()` conversion named the wrong joint type |
| `DEGENERATE_GEOMETRY` | Input points, counts, or extents describe no valid geometry |
| `NOT_A_UNIT_VECTOR` | A direction argument box3d requires to be normalized is not |
| `INDEX_OUT_OF_RANGE` | An index is negative or past the container's count |
| `CHILD_LIMIT_REACHED` | A compound definition names too many children |
| `NOT_A_PROXY` | A dynamic-tree id names no live proxy |
| `NO_HIT` | A cast, ray, or query found nothing, or `record::Player.diverge_frame()` found no divergence |
| `BUFFER_TOO_SMALL` | A caller buffer is shorter than the call requires |
| `FILE_UNAVAILABLE` | A path could not be read or written, or holds no data of the expected kind |
| `NOT_A_RECORDING` | A byte buffer carries no replayable recording |
| `NOT_A_COMPOUND` | Serialized compound data has the wrong version or length, or no node array that fits |

Each call's docstring lists the faults it returns and the exact condition for each.

## Symbol map

### [Library, math and identifiers](library_and_math.md)

Module: `b3`. Sources: `src/base.c3i`, `src/math.c3i`, `src/check.c3`.

Types: `AllocFn`, `FreeFn`, `AssertFn`, `LogFn`, `Version`, `Vec2`, `Vec3`, `Quat`, `Matrix3`, `Pos`, `WorldId`, `BodyId`, `ShapeId`, `JointId`, `ContactId`, `Transform`, `WorldTransform`, `Aabb`, `Plane`, `CosSin`, `SegmentDistanceResult`.

Constants: `HASH_INIT`.

Functions: `version`, `is_double_precision`, `byte_count`, `set_allocator`, `set_assert_fn`, `set_log_fn`, `ticks`, `milliseconds`, `milliseconds_and_reset`, `yield`, `sleep`, `length_units_per_meter`, `set_length_units_per_meter`, `hash`, `require_float_abi`, `max_manifold_points`, `is_avx2_available`, `point_to_segment_distance`, `compute_cos_sin`, `atan2`, `segment_distance`, `steiner`, `is_valid_float`, `is_valid_vec3`, `is_valid_position`, `is_valid_quat`, `is_valid_transform`, `is_valid_world_transform`, `is_valid_matrix3`, `is_valid_aabb`, `is_bounded_aabb`, `is_sane_aabb`, `is_valid_plane`, `make_quat_from_matrix`, `compute_quat_between_unit_vectors`, `line_distance`, `checked_access`, `checked_geometry`.

Methods: `WorldId.is_null`, `BodyId.is_null`, `ShapeId.is_null`, `JointId.is_null`, `ContactId.is_null`, `WorldId.checked`, `BodyId.checked`, `ShapeId.checked`, `JointId.checked`, `ContactId.checked`.


### [Worlds and events](worlds.md)

Module: `b3`. Sources: `src/world.c3i`, `src/event.c3i`.

Types: `FrictionFn`, `RestitutionFn`, `TaskFn`, `EnqueueTaskFn`, `FinishTaskFn`, `CreateDebugShapeFn`, `DestroyDebugShapeFn`, `Capacity`, `WorldDef`, `Profile`, `Counters`, `SensorBeginTouchEvent`, `SensorEndTouchEvent`, `ContactBeginTouchEvent`, `ContactEndTouchEvent`, `ContactHitEvent`, `BodyMoveEvent`, `JointEvent`, `PreSolveFn`, `CustomFilterFn`.

Constants: `MAX_WORLDS`, `MAX_WORKERS`, `MAX_RESTITUTION_ITERATIONS`.

Functions: `default_world_def`, `destroy_world`, `world_step`, `world_count`, `max_world_count`, `world_dump_memory_stats`, `create_world`.

Methods: `WorldId.is_valid`, `WorldId.gravity`, `WorldId.set_gravity`, `WorldId.restitution_threshold`, `WorldId.set_restitution_threshold`, `WorldId.hit_event_threshold`, `WorldId.set_hit_event_threshold`, `WorldId.maximum_linear_speed`, `WorldId.set_maximum_linear_speed`, `WorldId.contact_recycle_distance`, `WorldId.set_contact_recycle_distance`, `WorldId.set_contact_tuning`, `WorldId.enable_sleeping`, `WorldId.is_sleeping_enabled`, `WorldId.enable_continuous`, `WorldId.is_continuous_enabled`, `WorldId.enable_warm_starting`, `WorldId.is_warm_starting_enabled`, `WorldId.enable_speculative`, `WorldId.set_restitution_iterations`, `WorldId.restitution_iterations`, `WorldId.enable_restitution_propagation`, `WorldId.is_restitution_propagation_enabled`, `WorldId.set_isotropic_inertia_tolerance`, `WorldId.isotropic_inertia_tolerance`, `WorldId.enable_sse2_fallback`, `WorldId.worker_count`, `WorldId.set_worker_count`, `WorldId.awake_body_count`, `WorldId.profile`, `WorldId.counters`, `WorldId.max_capacity`, `WorldId.bounds`, `WorldId.user_data`, `WorldId.set_user_data`, `WorldId.set_friction_callback`, `WorldId.set_restitution_callback`, `ContactId.is_valid`, `JointId.is_valid`, `WorldId.set_pre_solve_callback`, `WorldId.set_custom_filter_callback`, `WorldId.contact_begin_touch_events`, `WorldId.contact_end_touch_events`, `WorldId.contact_hit_events`, `WorldId.sensor_begin_touch_events`, `WorldId.sensor_end_touch_events`, `WorldId.body_move_events`, `WorldId.joint_events`.


### [Bodies](bodies.md)

Module: `b3`. Sources: `src/body.c3i`.

Types: `BodyType`, `MotionLocks`, `MassData`, `BodyDef`.

Functions: `default_body_def`, `destroy_body`, `body_disable`, `body_enable`, `body_apply_force`, `body_apply_force_to_center`, `body_apply_torque`, `body_apply_linear_impulse`, `body_apply_linear_impulse_to_center`, `body_apply_angular_impulse`, `body_apply_mass_from_shapes`, `body_allow_fast_rotation`, `create_body`.

Methods: `BodyId.is_valid`, `BodyId.type`, `BodyId.set_type`, `BodyId.world`, `BodyId.is_enabled`, `BodyId.position`, `BodyId.rotation`, `BodyId.transform`, `BodyId.set_transform`, `BodyId.set_target_transform`, `BodyId.local_point`, `BodyId.world_point`, `BodyId.local_vector`, `BodyId.world_vector`, `BodyId.linear_velocity`, `BodyId.set_linear_velocity`, `BodyId.angular_velocity`, `BodyId.set_angular_velocity`, `BodyId.local_point_velocity`, `BodyId.world_point_velocity`, `BodyId.mass`, `BodyId.inverse_mass`, `BodyId.local_rotational_inertia`, `BodyId.world_inverse_rotational_inertia`, `BodyId.local_center`, `BodyId.world_center`, `BodyId.set_mass_data`, `BodyId.mass_data`, `BodyId.set_linear_damping`, `BodyId.linear_damping`, `BodyId.set_angular_damping`, `BodyId.angular_damping`, `BodyId.set_gravity_scale`, `BodyId.gravity_scale`, `BodyId.compute_aabb`, `BodyId.min_extent`, `BodyId.max_extent`, `BodyId.max_extent_origin`, `BodyId.set_safety_factor`, `BodyId.safety_factor`, `BodyId.set_name`, `BodyId.name`, `BodyId.set_user_data`, `BodyId.user_data`, `BodyId.is_awake`, `BodyId.set_awake`, `BodyId.enable_sleep`, `BodyId.is_sleep_enabled`, `BodyId.set_sleep_threshold`, `BodyId.sleep_threshold`, `BodyId.set_motion_locks`, `BodyId.motion_locks`, `BodyId.set_bullet`, `BodyId.is_bullet`, `BodyId.is_fast_rotation_allowed`, `BodyId.enable_contact_recycling`, `BodyId.is_contact_recycling_enabled`, `BodyId.enable_hit_events`.


### [Shapes](shapes.md)

Module: `b3`. Sources: `src/shape.c3i`, `src/hull.c3i`, `src/mesh.c3i`, `src/compound.c3i`.

Types: `Filter`, `SurfaceMaterial`, `ShapeType`, `ShapeDef`, `Sphere`, `Capsule`, `HullVertex`, `HullHalfEdge`, `HullFace`, `HullData`, `BoxHull`, `ScaledBox`, `MeshData`, `MeshDef`, `Mesh`, `HeightFieldData`, `HeightFieldDef`, `MeshQueryFn`, `CompoundData`, `CompoundSphereDef`, `CompoundCapsuleDef`, `CompoundHullDef`, `CompoundMeshDef`, `CompoundDef`, `CompoundCapsule`, `CompoundHull`, `CompoundMesh`, `CompoundSphere`, `ChildShape`, `CompoundQueryFn`.

Constants: `MAX_HULL_VERTICES`, `MAX_HULL_RING_STEPS`, `MAX_MESH_CONTACT_TRIANGLES`, `HEIGHT_FIELD_APRON_NEGATIVE_X`, `HEIGHT_FIELD_APRON_POSITIVE_X`, `HEIGHT_FIELD_APRON_NEGATIVE_Z`, `HEIGHT_FIELD_APRON_POSITIVE_Z`, `MAX_CHILD_SHAPES`.

Functions: `default_filter`, `default_surface_material`, `default_shape_def`, `destroy_shape`, `shape_apply_wind`, `create_sphere_shape`, `create_capsule_shape`, `create_hull_shape`, `create_transformed_hull_shape`, `make_cube_hull`, `make_box_hull`, `make_offset_box_hull`, `make_transformed_box_hull`, `make_scaled_box_hull`, `destroy_hull`, `create_hull`, `create_cylinder`, `create_cone`, `create_rock`, `create_complex_hull`, `clone_hull`, `clone_and_transform_hull`, `scale_box`, `destroy_mesh`, `destroy_height_field`, `query_mesh`, `query_height_field`, `create_grid_mesh`, `create_wave_mesh`, `create_torus_mesh`, `create_box_mesh`, `create_hollow_box_mesh`, `create_platform_mesh`, `create_mesh`, `create_height_field_grid`, `create_height_field_wave`, `create_height_field`, `dump_height_data`, `load_height_field`, `create_mesh_shape`, `create_height_field_shape`, `destroy_compound`, `query_compound`, `create_compound`, `validate_compound`, `create_baked_compound_shape`.

Methods: `Sphere.compute_mass_data`, `Sphere.compute_aabb`, `Capsule.compute_mass_data`, `Capsule.compute_aabb`, `ShapeId.is_valid`, `ShapeId.type`, `ShapeId.body`, `ShapeId.world`, `ShapeId.is_sensor`, `ShapeId.enable_sensor_events`, `ShapeId.are_sensor_events_enabled`, `ShapeId.set_name`, `ShapeId.name`, `ShapeId.set_user_data`, `ShapeId.user_data`, `ShapeId.set_density`, `ShapeId.density`, `ShapeId.set_friction`, `ShapeId.set_restitution`, `ShapeId.set_surface_material`, `ShapeId.friction`, `ShapeId.restitution`, `ShapeId.surface_material`, `ShapeId.mesh_material_count`, `ShapeId.filter`, `ShapeId.set_filter`, `ShapeId.enable_contact_events`, `ShapeId.are_contact_events_enabled`, `ShapeId.enable_pre_solve_events`, `ShapeId.are_pre_solve_events_enabled`, `ShapeId.enable_hit_events`, `ShapeId.are_hit_events_enabled`, `ShapeId.set_sphere`, `ShapeId.set_capsule`, `ShapeId.set_hull`, `ShapeId.aabb`, `ShapeId.compute_mass_data`, `ShapeId.closest_point`, `ShapeId.sphere`, `ShapeId.capsule`, `ShapeId.hull`, `ShapeId.mesh`, `ShapeId.height_field`, `ShapeId.compound`, `ShapeId.set_mesh`, `ShapeId.mesh_surface_material`, `ShapeId.set_mesh_material`, `HullData.compute_mass_data`, `HullData.compute_aabb`, `MeshData.tree_height`, `MeshData.compute_aabb`, `HeightFieldData.compute_aabb`, `CompoundData.compute_aabb`, `CompoundData.child_count`, `CompoundData.child`, `CompoundData.capsule`, `CompoundData.hull`, `CompoundData.mesh`, `CompoundData.sphere`, `CompoundData.materials`, `CompoundMesh.material_slots`, `ChildShape.material_slots`.


### [Joints](joints.md)

Module: `b3`. Sources: `src/joint.c3i`, `src/joint_distance.c3i`, `src/joint_motor.c3i`, `src/joint_mover.c3i`, `src/joint_parallel.c3i`, `src/joint_pogo.c3i`, `src/joint_prismatic.c3i`, `src/joint_revolute.c3i`, `src/joint_spherical.c3i`, `src/joint_weld.c3i`, `src/joint_wheel.c3i`.

Types: `JointType`, `JointDef`, `FilterJointDef`, `ConstraintTuning`, `DistanceJointId`, `DistanceJointDef`, `SpringForceRange`, `MotorJointId`, `MotorJointDef`, `MoverJointId`, `MoverJointDef`, `ParallelJointId`, `ParallelJointDef`, `PogoJointId`, `PogoJointDef`, `PrismaticJointId`, `PrismaticJointDef`, `RevoluteJointId`, `RevoluteJointDef`, `SphericalJointId`, `SphericalJointDef`, `WeldJointId`, `WeldJointDef`, `WheelJointId`, `WheelJointDef`.

Functions: `default_filter_joint_def`, `destroy_joint`, `joint_wake_bodies`, `create_filter_joint`, `default_distance_joint_def`, `create_distance_joint`, `default_motor_joint_def`, `create_motor_joint`, `default_mover_joint_def`, `create_mover_joint`, `default_parallel_joint_def`, `create_parallel_joint`, `default_pogo_joint_def`, `create_pogo_joint`, `default_prismatic_joint_def`, `create_prismatic_joint`, `default_revolute_joint_def`, `create_revolute_joint`, `default_spherical_joint_def`, `create_spherical_joint`, `default_weld_joint_def`, `create_weld_joint`, `default_wheel_joint_def`, `create_wheel_joint`.

Methods: `JointId.type`, `JointId.body_a`, `JointId.body_b`, `JointId.world`, `JointId.set_local_frame_a`, `JointId.local_frame_a`, `JointId.set_local_frame_b`, `JointId.local_frame_b`, `JointId.set_collide_connected`, `JointId.collide_connected`, `JointId.set_user_data`, `JointId.user_data`, `JointId.is_awake`, `JointId.constraint_force`, `JointId.constraint_torque`, `JointId.linear_separation`, `JointId.angular_separation`, `JointId.set_constraint_tuning`, `JointId.set_force_threshold`, `JointId.force_threshold`, `JointId.set_torque_threshold`, `JointId.torque_threshold`, `BodyId.joint_count`, `JointId.constraint_tuning`, `BodyId.joints`, `DistanceJointId.set_length`, `DistanceJointId.length`, `DistanceJointId.current_length`, `DistanceJointId.enable_limit`, `DistanceJointId.is_limit_enabled`, `DistanceJointId.set_length_range`, `DistanceJointId.min_length`, `DistanceJointId.max_length`, `DistanceJointId.enable_spring`, `DistanceJointId.is_spring_enabled`, `DistanceJointId.set_spring_hertz`, `DistanceJointId.spring_hertz`, `DistanceJointId.set_spring_damping_ratio`, `DistanceJointId.spring_damping_ratio`, `DistanceJointId.set_spring_force_range`, `DistanceJointId.enable_motor`, `DistanceJointId.is_motor_enabled`, `DistanceJointId.set_motor_speed`, `DistanceJointId.motor_speed`, `DistanceJointId.set_max_motor_force`, `DistanceJointId.max_motor_force`, `DistanceJointId.motor_force`, `JointId.as_distance`, `DistanceJointId.spring_force_range`, `MotorJointId.set_linear_velocity`, `MotorJointId.linear_velocity`, `MotorJointId.set_max_velocity_force`, `MotorJointId.max_velocity_force`, `MotorJointId.set_angular_velocity`, `MotorJointId.angular_velocity`, `MotorJointId.set_max_velocity_torque`, `MotorJointId.max_velocity_torque`, `MotorJointId.set_linear_hertz`, `MotorJointId.linear_hertz`, `MotorJointId.set_linear_damping_ratio`, `MotorJointId.linear_damping_ratio`, `MotorJointId.set_max_spring_force`, `MotorJointId.max_spring_force`, `MotorJointId.set_angular_hertz`, `MotorJointId.angular_hertz`, `MotorJointId.set_angular_damping_ratio`, `MotorJointId.angular_damping_ratio`, `MotorJointId.set_max_spring_torque`, `MotorJointId.max_spring_torque`, `JointId.as_motor`, `MoverJointId.set_linear_velocity`, `MoverJointId.linear_velocity`, `MoverJointId.set_max_velocity_force`, `MoverJointId.max_velocity_force`, `JointId.as_mover`, `ParallelJointId.set_spring_hertz`, `ParallelJointId.spring_hertz`, `ParallelJointId.set_spring_damping_ratio`, `ParallelJointId.spring_damping_ratio`, `ParallelJointId.set_max_torque`, `ParallelJointId.max_torque`, `JointId.as_parallel`, `PogoJointId.set_rest_length`, `PogoJointId.rest_length`, `PogoJointId.set_spring_hertz`, `PogoJointId.spring_hertz`, `PogoJointId.set_spring_damping_ratio`, `PogoJointId.spring_damping_ratio`, `PogoJointId.set_max_tension_force`, `PogoJointId.max_tension_force`, `PogoJointId.set_max_compression_force`, `PogoJointId.max_compression_force`, `PogoJointId.set_normal`, `PogoJointId.normal`, `PogoJointId.current_length`, `PogoJointId.current_speed`, `PogoJointId.impulse`, `JointId.as_pogo`, `PrismaticJointId.translation`, `PrismaticJointId.speed`, `PrismaticJointId.enable_limit`, `PrismaticJointId.is_limit_enabled`, `PrismaticJointId.lower_limit`, `PrismaticJointId.upper_limit`, `PrismaticJointId.set_limits`, `PrismaticJointId.enable_spring`, `PrismaticJointId.is_spring_enabled`, `PrismaticJointId.set_spring_hertz`, `PrismaticJointId.spring_hertz`, `PrismaticJointId.set_spring_damping_ratio`, `PrismaticJointId.spring_damping_ratio`, `PrismaticJointId.set_target_translation`, `PrismaticJointId.target_translation`, `PrismaticJointId.enable_motor`, `PrismaticJointId.is_motor_enabled`, `PrismaticJointId.set_motor_speed`, `PrismaticJointId.motor_speed`, `PrismaticJointId.set_max_motor_force`, `PrismaticJointId.max_motor_force`, `PrismaticJointId.motor_force`, `JointId.as_prismatic`, `RevoluteJointId.enable_motor`, `RevoluteJointId.is_motor_enabled`, `RevoluteJointId.set_motor_speed`, `RevoluteJointId.motor_speed`, `RevoluteJointId.motor_torque`, `RevoluteJointId.set_max_motor_torque`, `RevoluteJointId.max_motor_torque`, `RevoluteJointId.angle`, `RevoluteJointId.enable_limit`, `RevoluteJointId.is_limit_enabled`, `RevoluteJointId.lower_limit`, `RevoluteJointId.upper_limit`, `RevoluteJointId.set_limits`, `RevoluteJointId.enable_spring`, `RevoluteJointId.is_spring_enabled`, `RevoluteJointId.set_spring_hertz`, `RevoluteJointId.spring_hertz`, `RevoluteJointId.set_spring_damping_ratio`, `RevoluteJointId.spring_damping_ratio`, `RevoluteJointId.set_target_angle`, `RevoluteJointId.target_angle`, `JointId.as_revolute`, `SphericalJointId.enable_cone_limit`, `SphericalJointId.is_cone_limit_enabled`, `SphericalJointId.set_cone_limit`, `SphericalJointId.cone_limit`, `SphericalJointId.cone_angle`, `SphericalJointId.enable_twist_limit`, `SphericalJointId.is_twist_limit_enabled`, `SphericalJointId.lower_twist_limit`, `SphericalJointId.upper_twist_limit`, `SphericalJointId.set_twist_limits`, `SphericalJointId.twist_angle`, `SphericalJointId.enable_spring`, `SphericalJointId.is_spring_enabled`, `SphericalJointId.set_spring_hertz`, `SphericalJointId.spring_hertz`, `SphericalJointId.set_spring_damping_ratio`, `SphericalJointId.spring_damping_ratio`, `SphericalJointId.set_target_rotation`, `SphericalJointId.target_rotation`, `SphericalJointId.enable_motor`, `SphericalJointId.is_motor_enabled`, `SphericalJointId.set_motor_velocity`, `SphericalJointId.motor_velocity`, `SphericalJointId.motor_torque`, `SphericalJointId.set_max_motor_torque`, `SphericalJointId.max_motor_torque`, `JointId.as_spherical`, `WeldJointId.set_linear_hertz`, `WeldJointId.linear_hertz`, `WeldJointId.set_linear_damping_ratio`, `WeldJointId.linear_damping_ratio`, `WeldJointId.set_angular_hertz`, `WeldJointId.angular_hertz`, `WeldJointId.set_angular_damping_ratio`, `WeldJointId.angular_damping_ratio`, `JointId.as_weld`, `WheelJointId.enable_suspension`, `WheelJointId.is_suspension_enabled`, `WheelJointId.set_suspension_hertz`, `WheelJointId.suspension_hertz`, `WheelJointId.set_suspension_damping_ratio`, `WheelJointId.suspension_damping_ratio`, `WheelJointId.enable_suspension_limit`, `WheelJointId.is_suspension_limit_enabled`, `WheelJointId.lower_suspension_limit`, `WheelJointId.upper_suspension_limit`, `WheelJointId.set_suspension_limits`, `WheelJointId.enable_spin_motor`, `WheelJointId.is_spin_motor_enabled`, `WheelJointId.set_spin_motor_speed`, `WheelJointId.spin_motor_speed`, `WheelJointId.set_max_spin_torque`, `WheelJointId.max_spin_torque`, `WheelJointId.spin_speed`, `WheelJointId.spin_torque`, `WheelJointId.enable_steering`, `WheelJointId.is_steering_enabled`, `WheelJointId.set_steering_hertz`, `WheelJointId.steering_hertz`, `WheelJointId.set_steering_damping_ratio`, `WheelJointId.steering_damping_ratio`, `WheelJointId.set_max_steering_torque`, `WheelJointId.max_steering_torque`, `WheelJointId.enable_steering_limit`, `WheelJointId.is_steering_limit_enabled`, `WheelJointId.lower_steering_limit`, `WheelJointId.upper_steering_limit`, `WheelJointId.set_steering_limits`, `WheelJointId.set_target_steering_angle`, `WheelJointId.target_steering_angle`, `WheelJointId.steering_angle`, `WheelJointId.steering_torque`, `JointId.as_wheel`.


### [Queries and casts](queries_and_casts.md)

Module: `b3`. Sources: `src/query.c3i`, `src/cast.c3i`.

Types: `QueryFilter`, `RayResult`, `ShapeProxy`, `CastOutput`, `WorldCastOutput`, `BodyCastResult`, `TreeStats`, `ManifoldPoint`, `Manifold`, `ContactData`, `ClosestPoint`, `CastResultFn`, `OverlapResultFn`, `CastHit`, `RayCastInput`, `ShapeCastInput`.

Constants: `MAX_MANIFOLD_POINTS`, `MAX_SHAPE_CAST_POINTS`.

Functions: `default_query_filter`, `body_overlap_shape`, `world_cast_ray`, `world_cast_shape`, `world_overlap_aabb`, `world_overlap_shape`, `world_cast_ray_closest`, `shape_ray_cast`, `body_cast_ray`, `body_cast_shape`, `world_cast_ray_all`, `world_cast_shape_all`, `world_overlap_aabb_all`, `world_overlap_shape_all`, `ray_cast_sphere`, `ray_cast_hollow_sphere`, `ray_cast_capsule`, `ray_cast_hull`, `ray_cast_compound`, `ray_cast_mesh`, `ray_cast_height_field`, `shape_cast_sphere`, `shape_cast_capsule`, `shape_cast_hull`, `shape_cast_compound`, `shape_cast_mesh`, `shape_cast_height_field`, `overlap_sphere`, `overlap_capsule`, `overlap_hull`, `overlap_compound`, `overlap_mesh`, `overlap_height_field`.

Methods: `ContactId.data`, `BodyId.contact_capacity`, `ShapeId.contact_capacity`, `BodyId.shape_count`, `ShapeId.sensor_capacity`, `BodyId.closest_point`, `ContactData.contact_manifolds`, `Manifold.contact_points`, `BodyId.contact_data`, `ShapeId.contact_data`, `BodyId.shapes`, `ShapeId.sensor_data`, `RayCastInput.is_valid`.


### [Standalone geometry](geometry.md)

Module: `b3`. Sources: `src/collide.c3i`, `src/distance.c3i`.

Types: `Triangle`, `FeaturePair`, `Point2D`, `LocalManifoldPoint`, `CollisionManifold`, `CollisionManifoldView`, `DistanceInput`, `DistanceOutput`, `ShapeCastPairInput`, `Sweep`, `ToiInput`, `ToiState`, `ToiOutput`.

Functions: `collide_spheres`, `collide_capsule_and_sphere`, `collide_hull_and_sphere`, `collide_capsules`, `collide_hull_and_capsule`, `collide_hulls`, `collide_triangle_and_sphere`, `collide_triangle_and_capsule`, `collide_triangle_and_hull`, `hull_2d`, `simplify_hull_2d`, `shape_distance`, `shape_cast`, `time_of_impact`.

Methods: `CollisionManifold.contact_points`, `Sweep.transform_at`.


### [Dynamic tree](dynamic_tree.md)

Module: `b3`. Sources: `src/tree.c3i`.

Types: `TreeNode`, `TreeProxy`, `DynamicTree`, `TreeQueryFn`, `TreeClosestFn`, `ClosestSearch`, `BoxCastInput`, `TreeRayCastFn`, `TreeBoxCastFn`.

Functions: `dynamic_tree_create`, `dynamic_tree_destroy`, `dynamic_tree_save`, `dynamic_tree_rebuild`, `dynamic_tree_query`, `dynamic_tree_ray_cast`, `dynamic_tree_box_cast`, `dynamic_tree_create_proxy`, `dynamic_tree_destroy_proxy`, `dynamic_tree_move_proxy`, `dynamic_tree_enlarge_proxy`, `dynamic_tree_query_closest`, `load_tree`.

Methods: `DynamicTree.height`, `DynamicTree.area_ratio`, `DynamicTree.root_bounds`, `DynamicTree.byte_count`, `DynamicTree.set_category_bits`, `DynamicTree.category_bits`, `DynamicTree.user_data`, `DynamicTree.aabb`.


### [Character mover](character_mover.md)

Module: `b3`. Sources: `src/mover.c3i`.

Types: `ExplosionDef`, `PlaneResult`, `CollisionPlane`, `PlaneSolverResult`, `BodyPlaneResult`, `BodyToiResult`, `PlaneResultFn`, `MoverFilterFn`.

Functions: `default_explosion_def`, `world_collide_mover`, `world_cast_mover`, `solve_planes`, `clip_vector`, `body_collide_mover`, `body_time_of_impact_mover`, `world_explode`.


### [Debug draw](debug_draw.md)

Module: `b3::draw`. Sources: `src/draw.c3i`.

Types: `HexColor`, `DebugShape`, `DrawShapeFn`, `DrawSegmentFn`, `DrawTransformFn`, `DrawPointFn`, `DrawSphereFn`, `DrawCapsuleFn`, `DrawBoundsFn`, `DrawBoxFn`, `DrawStringFn`, `DebugDraw`.

Constants: `GRAPH_COLOR_COUNT`.

Functions: `default_debug_draw`, `world_draw`, `graph_color`.


### [Recording and replay](recording.md)

Module: `b3::record`. Sources: `src/record.c3i`.

Types: `Recording`, `Player`, `PlayerInfo`, `QueryType`, `QueryInfo`, `QueryHit`.

Functions: `create_recording`, `destroy_recording`, `world_stop_recording`, `destroy_player`, `player_step_frame`, `player_sub_step_frame`, `player_restart`, `player_seek_frame`, `player_has_diverged`, `player_draw_frame_queries`, `world_start_recording`, `save_recording_to_file`, `load_recording`, `validate_replay`, `create_player`.

Methods: `Player.world_id`, `Player.frame`, `Player.frame_count`, `Player.is_at_end`, `Player.is_at_pre_step`, `Player.info`, `Player.set_worker_count`, `Player.set_keyframe_policy`, `Player.keyframe_budget`, `Player.keyframe_min_interval`, `Player.keyframe_interval`, `Player.keyframe_bytes`, `Player.body_count`, `Player.set_debug_shape_callbacks`, `Player.frame_query_count`, `Recording.data`, `Player.diverge_frame`, `Player.body_id`, `Player.frame_query`, `Player.frame_query_hit`.
