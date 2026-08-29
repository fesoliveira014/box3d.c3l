# Style Guide

Mirror of the Notion page [Style Guide (docs/style.md)](https://app.notion.com/p/3bccb7903a5881089469c7001fa88d7c) — the mandatory style baseline for all manually maintained C3 and GLSL. Keep the two in sync. `AGENTS.md`/`CLAUDE.md` §project rules may refine or strengthen these rules; they never relax them. Written for agents, readable by humans: every rule is an imperative.

## 1. Scope & precedence

- Applies to every `.c3`, `.c3i`, and `.glsl` file, hand-written or reviewed. Generated code is exempt from formatting rules but not from layout pins.
- Precedence: this baseline → project refinements → the configured compiler (C3 0.8.3) — later entries may narrow, never widen.
- Skills `c3-expert`, `c3-style`, `c3-bindings`, `shader-dev` are loaded before writing or reviewing code. A review without them is not valid.

## 2. Naming

| Kind | Case | Examples |
|---|---|---|
| Variables, fields, parameters | `snake_case` | `page_index`, `edit_version` |
| Functions | `snake_case` | `sample_cell`, `publish_deltas` |
| Structs, enums, typedefs, aliases | `PascalCase` | `PageMeta`, `BuildKey` |
| Constants, enum values, faults | `SCREAMING_SNAKE_CASE` | `BRICK_CELLS`, `SLOT_TABLE_FULL` |
| Modules | lowercase under a single root | `vox::gfx`, `vox::world::gen` |
| Files | `snake_case.c3` | `rank9.c3`, `shadow_cache.c3` |
| Shaders | `<name>.<stage>.glsl` | `dda.comp.glsl`; shared includes plain `.glsl` |

## 3. Layout & formatting

- K&R braces: opening brace on the same line as its declaration or statement, closing brace on its own line. 4-space block indentation. Every file type, no exceptions beyond the one below.
- Wrapped function declarations: a declaration spanning lines ends its first line with `(`; each parameter on its own line with exactly two leading spaces and a trailing comma (final parameter included); `)` starts its own line, followed by attributes and `{`. Never align under the first parameter; never leave a parameter on the declaration line. This two-space continuation is the sole exception to 4-space indentation.
- Calls with ≥4 arguments: named arguments, one per line, trailing comma.
- Struct initializers: every supplied field in every non-empty initializer uses `.field = value` — nested, returned, argument, assignment, test, and compound/cast literals included. Empty `{}` stays allowed; arrays and vectors stay positional.
- Never run `c3fmt` — it is line-length-aware but not argument-aware and breaks the ≥4-arg format and one-fault-per-line faultdefs. Hand-format. Fix stray style only while touching code for another reason; never a whitespace-only commit.
- File order: typedefs → aliases → constants → enums/bitstructs → structs → struct methods → free functions.

## 4. Errors, faults, assertions

- Fallible operations return optionals (`?`) with named faults — never sentinel values, never null-as-error.
- Propagate with `!`; handle with `if (catch err = ...)`; clean up with `defer`; failure-only cleanup uses `defer catch destroy`.
- `@unwrap` / `@fatal` only after init completes (it exits, bypassing defers). Init paths use catch + log + return.
- One `faultdef` file per domain (`voxel/faults.c3`, `stream/faults.c3`, …), one fault per line.
- Runtime `assert` only in test code. Production code returns a fault or succeeds — callers must be able to handle failure. Compile-time `$assert` layout pins are required, not optional.
- `unreachable()` marks control flow that is impossible by construction — never a failure that can occur at runtime. Every operational failure (input, state, allocation, I/O, device) returns a named fault through an optional; `unreachable()` standing in for one is a review defect. Compile-time `$assert` stays appropriate for ABI and constant checks because it creates no runtime failure path.

## 5. GPU, GLSL & compute discipline

- The ABI is generated: `abi/*.abi` → `gen_abi.py` → C3 + GLSL twins with `$assert` size/offset pins; `--check` gates the build. A layout change regenerates both sides in the same change; never hand-edit generated files.
- Constants mirrored into GLSL outside the codegen name their twin in a comment: `// mirrored as AO_RADIUS_MAX in ao.comp.glsl`.
- The binding contract is data: write it down on both the C3 and GLSL side; never infer one side from the other.
- Textures keep one resting state (SAMPLED); each pass transitions away and back, so every barrier `before` is exact. Bind pipelines before `begin_render_pass`.
- Spans name every queue role that touches them — `.graphics = true` even on compute-only allocations.
- Dispatch shape, workgroup layout, and barrier placement follow gpu.c3l's `docs/shader_abi.md` and `docs/cookbook.md`; `shader-dev` owns technique (marching, AO, shadows), not Vulkan discipline.
- `--gpu_validation=true` in every development run.

## 6. Ownership & lifecycle

- Project-owned resources use free functions: `create_x`/`destroy_x`, `allocate_x`/`free_x` — never `Type.create`.
- `X` owns; `XView` borrows; views have no destructor. Only load/build functions produce owners — double-free is unrepresentable by construction.
- Anything the GPU reads beyond the frame arena's two-frame recycle owns its allocation (pool-slot ownership).
- Every handle and vector alias lives in `src/types.c3`.

## 7. Comments & docstrings

- Code is self-documenting: names carry meaning, structure carries flow.
- Exactly two comment forms: `<* ... *>` docstrings above public functions — the contract plus the one dangerous property ("exits, bypassing defers"), never a narration of the body — and short *why* comments on non-trivial code. A comment describing *what* the code does is a defect: delete it and improve the names.
- Tunable constants document their why and cost model: `// 384 K bricks ≈ 63 MiB — the resident-memory lever`.
- Development terminology never appears in code: no PR numbers, step numbers, change IDs, issue refs, or plan vocabulary in identifiers, filenames, comments, docstrings, test names, or string literals.

## 8. Verification

- `c3c build <target>` + `c3c test <target>` before every commit; broken builds are never committed.
- Tests are ordinary C3 sources under `test/`, one file per group — no bespoke runner.
