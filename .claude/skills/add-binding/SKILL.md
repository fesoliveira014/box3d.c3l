---
name: add-binding
description: Bind a box3d C symbol into the b3 C3 module across the three layers (extern in box3d.c3i, optional-returning wrapper in box3d.c3, faults in box3d_check.c3). Use when binding a new box3d function, ID type, struct, or enum — e.g. "bind b3CreateWorld", "/add-binding b3Body_SetTransform", "bind the shape API". Reads the real box3d header for exact signatures, then applies this repo's prefix-stripping, method-syntax, @cname, $assert layout-pin, and error-as-fault rules.
---

# Add a box3d binding

Bind the requested box3d symbol(s) into the `b3` module. Input: `$ARGUMENTS` (a C symbol, a feature area, or a description). If empty, ask what to bind.

Before writing any C3: invoke the `c3-expert`, `c3-style`, and `c3-bindings` skills, and read `docs/style.md` and `CLAUDE.md`. Those are the source of truth and override anything here.

## 1. Read the real signature — never bind from memory

The box3d C headers are the only source of truth. A transcribed parameter or return type that is wrong corrupts memory silently across the ABI — the worst and hardest-to-find binding bug.

- Headers live in the box3d submodule under `include/box3d/`. If the submodule is not checked out, run `git submodule update --init` (or ask the user where box3d is) rather than guessing.
- Grep for the specific symbol; do not read whole headers:
  - function → `grep -rnA6 'b3CreateWorld' include/box3d/`
  - ID / struct / enum → `grep -rnA25 'typedef struct b3WorldDef' include/box3d/`
- Copy the exact parameter list and return type out of the header verbatim.

## 2. Translate into `box3d.c3i`

- **The `b3` prefix is the module name; strip it from every identifier and keep the real symbol verbatim in `@cname`.**
- **Functions** → `snake_case`. A `b3Type_Method` C name becomes method syntax on that type; a bare `b3Verb...` constructor/destructor becomes a method on the type it produces or consumes where that reads naturally, otherwise a free function.

  ```c3
  extern fn WorldId create_world(WorldDef* def) @cname("b3CreateWorld");
  extern fn void WorldId.destroy(self) @cname("b3DestroyWorld");
  extern fn void BodyId.set_transform(self, Pos position, Quat rotation) @cname("b3Body_SetTransform");
  ```

- **Types** → `PascalCase`, `b3` stripped. The IDs (`b3WorldId`, `b3BodyId`, `b3ShapeId`, `b3JointId`, `b3ContactId`) are **value structs passed and returned by value**, not opaque pointers — declare their fields exactly as the header has them. Use `@opaque` or `inline void*` only for a type that genuinely is a pointer handle in C (`b3RecPlayer*` and the like), and only after confirming it in the header.
- **Structs C3 reads** are declared in full, field for field, in header order.
- **Constants / enum values** → `SCREAMING_SNAKE_CASE`, prefix stripped. A closed C set becomes a C3 `enum`; a flag-bits set becomes a `bitstruct X : uint { bool field : 0; ... }`.
- Keep the extern layer faithful to C: raw return types, raw out-parameters, no interpretation. No `@builtin` on any declaration.
- File order per `docs/style.md`: typedefs → aliases → constants → enums/bitstructs → structs → struct methods → free functions.

## 3. Pin every declared struct's layout

Add `$assert T::size == N;` immediately after each fully-declared struct (0.8.x syntax — `::size`, not `.sizeof`). Get `N` from `scripts/build-box3d.sh`, never from an ad-hoc probe, so later drift is caught by `--check`:

1. Add the C type name to `scripts/abi-types.txt` (one type per line; append a comma-separated field list — `b3WorldDef fieldA,fieldB` — only if the binding also needs individual field offsets pinned, e.g. for a struct nothing bound reads field-by-field).
2. Run `./scripts/build-box3d.sh --update` from the repository root.
3. Read the size (and any field offsets) for the type out of `scripts/abi-sizes.txt`.

A mismatch here is exactly the silent-corruption bug the assert exists to catch. Never guess `N`.

## 4. Add the wrapper in `box3d.c3` — this is where errors become faults

Callers use the wrapper layer, not the externs. It must not leak C's error conventions:

- Fallible operations return an optional (`?`) with a named fault. **Never** return null-as-error, a sentinel, or a raw status code.
- An invalid or zero ID coming back from C becomes a fault, not a value the caller has to test. Route it through that type's `.checked()` macro in `box3d_check.c3` (add one, following `WorldId.checked()`/`BodyId.checked()`, if the type doesn't have one yet); add a new `faultdef` there (one fault per line) when no existing one fits.
- Out-parameters become return values. A `pointer + count` pair becomes a slice.
- Ownership follows `docs/style.md` §6: `create_x`/`destroy_x` free functions, `defer catch destroy` for failure-only cleanup.

Struct initializers use `.field = value` for every supplied field; calls with ≥4 arguments use named arguments, one per line, trailing comma.

## 5. Verify

`module b3` spans `box3d.c3i`, `box3d.c3`, and `box3d_check.c3`, so a single-file invocation cannot see the declarations its siblings provide — compile them together in one invocation.

```sh
c3c compile-only --no-obj box3d.c3i box3d.c3 box3d_check.c3
```

Fix every error before finishing, and delete the `obj/` directory the check leaves behind. Note the compiler on PATH may be older than the 0.8.3 target this repo declares — if an error looks like a version mismatch, say so rather than working around it.

## 6. Scope and report

Bind only what was asked plus the types those signatures need to compile. box3d is large; do not mirror it wholesale. When done, report: which symbols you added, which structs you declared and the `$assert` sizes you pinned (and how you obtained them), which faults you added, and anything in the header you could not resolve.
