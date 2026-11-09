# Documentation

Consumer guide to `b3`, the C3 binding for box3d. Docstrings in `src/*.c3i` and `src/*.c3` are
the authority for exact signatures and fault lists.

## Learn

1. [Getting started](getting_started.md): install, then a world with a falling body and its
   contact events.
2. [Architecture](architecture.md): modules, how C names map to C3, identifiers, definitions,
   ownership, faults, threading, and the float ABI. Read this before designing around the API.
3. [Cookbook](cookbook.md): recipes for stepping, forces, ray casts, overlaps, events, joints,
   meshes, the character mover, debug draw, recording, and library hooks.
4. [Features and limitations](features_and_limitations.md): what is bound, what is
   [not bound](features_and_limitations.md#not-bound) and why, targets, and fixed limits.

## Reference

The [API index](api/index.md) explains the shared conventions and fault set and lists every public
symbol by domain. Domain pages:

- [Library, math and identifiers](api/library_and_math.md)
- [Worlds and events](api/worlds.md)
- [Bodies](api/bodies.md)
- [Shapes](api/shapes.md)
- [Joints](api/joints.md)
- [Queries and casts](api/queries_and_casts.md)
- [Standalone geometry](api/geometry.md)
- [Dynamic tree](api/dynamic_tree.md)
- [Character mover](api/character_mover.md)
- [Debug draw](api/debug_draw.md) (`b3::draw`)
- [Recording and replay](api/recording.md) (`b3::record`)

## Troubleshoot

- Call `b3::require_float_abi()` once at startup. It refuses a box3d built with double precision,
  which would corrupt every call.
- On Windows, build with `"wincrt": "static"`. A dynamic-CRT project fails to link with
  duplicate-symbol errors.
- box3d ships as a Release build, so its own assertions are compiled out. The wrappers guard the
  arguments that would corrupt memory, but not stale identifiers or freed geometry; see
  [architecture](architecture.md). Route identifiers through `.checked()` to have a null
  identifier reported as a fault.
- Install a log hook with `b3::set_log_fn` to see box3d's messages.
