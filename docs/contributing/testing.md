# Testing

Contributing › Testing

`test/` is a standalone consumer project that exercises the bindings. It is not part of the
shipped library: `manifest.json` never references it, so consumers never inherit its
dependencies. `test/libs/b3.c3l` is a symlink to the repository root, named for the manifest's
`provides` because `c3c project fetch` looks for `<search-path>/<dependency>.c3l`.

## Verification

Run before every commit. The `verify-bindings` skill runs the same steps.

```sh
./scripts/build-box3d.sh --check           # build box3d, fail on layout drift
c3c compile-only --no-obj src/*.c3i src/*.c3 && rm -rf obj
(cd test && c3c build smoke && ./build/smoke)
(cd test && c3c test unit)                 # BOX3D_CHECKED off
(cd test && c3c test unit-checked)         # BOX3D_CHECKED on
./scripts/audit-comments.sh --gate
./scripts/audit-assertions.sh
./scripts/check-docs.sh
```

- `module b3` spans most of `src/`, so compile the package in one invocation with a glob; a
  single file cannot see its siblings' declarations.
- `smoke` references nothing from `b3`, so it links even with a wrong `@cname`. It proves the
  package compiles against a real consumer's dependency setup, nothing more.
- `unit` links only if every `@cname` a test calls resolves in `libbox3d.a`. An extern nothing
  calls emits no undefined reference, so its symbol is never checked. Every bound symbol must be
  called by at least one test.
- `check-docs.sh` fails when a public symbol is missing from `docs/api/index.md` or a relative
  link in the docs does not resolve.

## Coverage audits

Every `@cname` resolves to an exported symbol:

```sh
grep -hoE '@cname\("[^"]+"\)' src/*.c3i | sed -E 's/@cname\("//; s/"\)//' | sort -u > /tmp/cnames.txt
nm -g --defined-only linked-libs/linux-x64/libbox3d.a | awk '$2=="T"{print $3}' | sort -u > /tmp/libsyms.txt
comm -23 /tmp/cnames.txt /tmp/libsyms.txt   # must print nothing
```

Every bound extern is reached by a test:

```sh
grep -hoE '^extern fn [A-Za-z0-9_*]+ ([A-Za-z0-9_]+\.)?[a-z0-9_]+\(' src/*.c3i \
  | sed -E 's/^extern fn [A-Za-z0-9_*]+ //; s/\(//; s/^[A-Za-z0-9_]+\.//' | sort -u \
  | while read s; do
      [ "$(grep -rhoE "\.$s\(|\b$s\(" test/ src/*.c3 | wc -l)" -eq 0 ] && echo "$s"
    done                                      # must print nothing
```

## Leaks shift every later test

C3 runs every test in one process, and its panic path skips `defer`. A test that panics after
creating a world leaks it, and every later test that counts live worlds or bytes fails against a
shifted baseline. **One direct failure plus a burst of unrelated `world_count()`/`byte_count()`
failures is one leaked world, not several regressions.** Fix the first failure.

- box3d-allocated geometry (`MeshData`, `HeightFieldData`) shares the `byte_count()` baseline.
  Every fixture `defer`s its destroy on the line after the creation.
- Several box3d refusal paths free only part of what they took. Never reach a refusal that has not
  been measured allocation-free; the usual instinct to exercise the failure path is inverted here.
- In a safe build, `[&in]` panics on a null argument before the wrapper body runs, taking the
  test's world with it. The explicit null guard behind it is for release builds.
- A test that changes a process-wide setting (allocator, length unit) restores it
  before its assertions, not in a `defer`.
- Allocation assertions compare against a baseline taken at the top of the test, never zero.

## Assertions that can fail

- Assert the payload, not that a call returned: which shapes an event names, that a normal has
  unit length and a point does not. That pair catches a `Pos`/`Vec3` transposition, which compiles
  and produces plausible numbers.
- Physical quantities get a two-sided band. A one-sided bound passes for a wrong binding.
- Exact comparison is correct for exactly representable defaults (60, 2, 0, `float::max`); say so
  in the test.
- A test that distinguishes two contracts uses a value at which they differ. Revolute clamps limits
  to ±0.99π while prismatic does not, so `set_limits(3, -3)` cannot tell them apart and
  `set_limits(10, -10)` can.
- No fixture hands out a payload equal to its container's zero. An assertion that expects the zero
  passes with the write deleted.
- An empty-buffer test uses a buffer nothing has written to, and asserts it is still zero.
- Never assert box3d's definition cookie against its literal value; it lives in a private header.
  Compare a zeroed definition against a default one.

## Proving a guard is tested

- When a new branch returns a different fault from its siblings, temporarily change its fault and
  confirm the new test fails while the old ones pass.
- When two conditions share one fault (`index < 0 || index >= count`), delete one half at a time.
  Mutating the shared fault fires on whichever half runs first and proves nothing about the other.
- A guard whose deletion breaks no test is either dead or untested. Run the pass over every new
  branch.
- A mutation must be representable at the magnitude under test (`+= 1.0f` on 1e9 does nothing),
  and a hardcode mutation only tests a parameter the fixtures vary.
- A guard can be correct and untestable when box3d refuses the same inputs allocation-free and
  only diverges where it would write through a failed allocation. Keep it and say so in the test
  file.
