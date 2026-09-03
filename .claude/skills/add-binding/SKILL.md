---
name: add-binding
description: Bind a box3d C symbol into the b3 C3 module across the three layers (extern in src/<area>.c3i, optional-returning wrapper in src/<area>.c3, faults in src/check.c3). Use when binding a new box3d function, ID type, struct, or enum — e.g. "bind b3CreateWorld", "/add-binding b3Body_SetTransform", "bind the shape API". Reads the real box3d header for exact signatures, then applies this repo's prefix-stripping, method-syntax, @cname, $assert layout-pin, and error-as-fault rules.
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

## 2. Translate into `src/<area>.c3i`

- **The `b3` prefix is the module name; strip it from every identifier and keep the real symbol verbatim in `@cname`.**
- **Functions** → `snake_case`. A `b3Type_Method` C name becomes method syntax on that type; a bare `b3Verb...` constructor/destructor becomes a method on the type it produces or consumes where that reads naturally, otherwise a free function.

  ```c3
  extern fn WorldId create_world_raw(WorldDef* def) @cname("b3CreateWorld") @private;
  extern fn void WorldId.destroy(self) @cname("b3DestroyWorld");
  extern fn void BodyId.set_transform(self, Pos position, Quat rotation) @cname("b3Body_SetTransform");
  ```

  When a wrapper in `src/<area>.c3` takes the plain name a C symbol maps to (`create_world`), the extern it wraps takes a `_raw` suffix (`create_world_raw`) and is marked `@private` — the wrapper is the only public way to reach it, so a consumer cannot go around whatever validation the wrapper does.

- **Types** → `PascalCase`, `b3` stripped. The IDs (`b3WorldId`, `b3BodyId`, `b3ShapeId`, `b3JointId`, `b3ContactId`) are **value structs passed and returned by value**, not opaque pointers — declare their fields exactly as the header has them. Use `@opaque` or `inline void*` only for a type that genuinely is a pointer handle in C (`b3RecPlayer*` and the like), and only after confirming it in the header.
- **Structs C3 reads** are declared in full, field for field, in header order.
- **Constants / enum values** → `SCREAMING_SNAKE_CASE`, prefix stripped. A closed C set becomes a C3 `enum`; a flag-bits set becomes a `bitstruct X : uint { bool field : 0; ... }`.
- Keep the extern layer faithful to C: raw return types, raw out-parameters, no interpretation. No `@builtin` on any declaration.
- File order per `docs/style.md`: typedefs → aliases → constants → enums/bitstructs → structs → struct methods → free functions — except within a `src/<area>.c3i`, where `CLAUDE.md`'s carve-out applies instead: declarations are grouped by area, not by declaration kind.

- **Which file, and which module.** The symbol goes in the pair for its area — `b3Body_*` in `src/body.c3i` + `src/body.c3`, and a genuinely new area gets a new pair, following `src/world.*`. Where an extern has no wrapper, it goes with the type it is a method on. Almost every file is `module b3;`; the two exceptions are `src/draw.*` (`module b3::draw;`) and `src/record.*` (`module b3::record;`), and a new symbol joins one of those only if it is part of debug draw or of recording and replay. **Do not create a third submodule** — `CLAUDE.md` records why the rest of the API is flat. Inside a submodule, a fault named in a `@return?` must be written `b3::NAME`; the compiler enforces it.

## 3. Pin every declared struct's layout

Layout pins are generated, never hand-written, and they live in `src/layout.c3` rather than beside the struct — writing one by hand puts a number in the repository that nothing measured.

1. Add the C type name to `scripts/abi-types.txt`, one per line. Append a comma-separated field list — `b3WorldDef fieldA,fieldB` — to pin individual field offsets as well; list every field of the struct, not a subset, since the generated pins are positional.
2. Run `./scripts/build-box3d.sh --update` from the repository root. It probes the real headers, rewrites `scripts/abi-sizes.txt`, and regenerates `src/layout.c3` with a size, alignment and per-field pin for every listed type, converting each C name to its C3 spelling. Where that conversion cannot reach the C3 name, write it outright as `b3X=Y` — the `b3Rec*` family does, because the transformation would restore the prefix `b3::record` already carries.
3. Check the generated pins compile: a failure means the C3 struct disagrees with the header, which is the whole point.

`./scripts/build-box3d.sh --check` then fails if the built library drifts from the recorded sizes, or if `src/layout.c3` is stale relative to the probe.

A mismatch here is exactly the silent-corruption bug the assert exists to catch. Never guess `N`.

## 4. Add the wrapper — this is where errors become faults

Callers use the wrapper layer, not the externs. Each area keeps its own wrapper beside its
declarations: `src/base.c3` holds the library-level hooks, `src/world.c3` the world wrapper. The
wrapper must not leak C's error conventions:

- Fallible operations return an optional (`?`) with a named fault. **Never** return null-as-error, a sentinel, or a raw status code.
- An invalid or zero ID coming back from C becomes a fault, not a value the caller has to test. Route it through that type's `.checked()` macro in `src/check.c3` (add one, following `WorldId.checked()`/`BodyId.checked()`, if the type doesn't have one yet); add a new `faultdef` there (one fault per line) when no existing one fits.
- Out-parameters become return values. A `pointer + count` pair becomes a slice.
- Ownership follows `docs/style.md` §6: `create_x`/`destroy_x` free functions, `defer catch destroy` for failure-only cleanup — except where `CLAUDE.md`'s carve-outs apply: a constructor with a natural receiver already at hand (`world.create_body(def)`) is a method, not a free function; only a constructor with no existing receiver, like `create_world`, stays free.

Struct initializers use `.field = value` for every supplied field; calls with ≥4 arguments use named arguments, one per line, trailing comma.

## 5. Verify

`module b3` spans most of `src/`, so a single-file invocation cannot see the declarations its siblings provide — compile them together in one invocation.

```sh
c3c compile-only --no-obj src/*.c3i src/*.c3
```

Fix every error before finishing, and delete the `obj/` directory the check leaves behind. Note the compiler on PATH may be older than the 0.8.3 target this repo declares — if an error looks like a version mismatch, say so rather than working around it.

## 6. Scope and report

Bind only what was asked plus the types those signatures need to compile. box3d is large; do not mirror it wholesale. When done, report: which symbols you added, which structs you declared and the `$assert` sizes you pinned (and how you obtained them), which faults you added, and anything in the header you could not resolve.
