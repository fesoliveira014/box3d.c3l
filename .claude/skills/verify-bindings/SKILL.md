---
name: verify-bindings
description: Compile-check the b3 C3 binding bundle and build the test/ consumer project against it. Use before every commit, after editing any .c3/.c3i in this repo, or when asked to verify, check, or test the bindings.
---

# Verify the box3d bindings

This package has no `project.json` and no standalone build, so verification is four steps: compile-check the binding package, build the consumer's `smoke` target, run the consumer's `unit` and `unit-checked` test targets, then check the ABI pins.

## 1. Compile-check the binding package

`module b3` spans every `.c3`/`.c3i` file at the package root, so a single-file invocation cannot see the declarations its siblings provide — compile them together in one invocation, with a glob so the next file added is never silently skipped.

```sh
cd /home/fesol/source/repos/box3d.c3l
c3c compile-only --no-obj *.c3i *.c3
rm -rf obj
```

This catches syntax and type errors but not linkage or ABI problems.

## 2. Build the consumer's smoke target

```sh
cd /home/fesol/source/repos/box3d.c3l/test
c3c build smoke
./build/smoke
```

`smoke` (`test/src/main.c3`) references nothing from the `b3` module, so it emits no extern symbol and links even with a corrupted `@cname` — it proves the binding package compiles standalone against a real consumer's dependency setup, nothing about linkage or ABI correctness.

This step needs `linked-libs/linux-x64/` populated. If it is empty, the native library has not been built: run `scripts/build-box3d.sh` from the repository root and report if the submodule is not checked out rather than skipping the step silently.

## 3. Run the consumer's test targets

```sh
cd /home/fesol/source/repos/box3d.c3l/test
c3c test unit
c3c test unit-checked
```

This gates every `@cname` that a test actually calls — `unit` cannot link if a called binding's `@cname` fails to resolve to the real symbol in `linked-libs/linux-x64/libbox3d.a`. It does *not* gate a `@cname` nothing calls: an `extern fn` linked but never invoked emits no undefined reference, so a wrong symbol on an uncalled binding still links and still passes. The rule that keeps this gate meaningful: every bound symbol should be called by at least one test. `unit-checked` builds the same tests with the `BOX3D_CHECKED` feature on, exercising the `.checked()` path. Neither step 1 nor step 2 catches a wrong `@cname` — only a test that calls the binding does.

## 4. Check the ABI pins

```sh
cd /home/fesol/source/repos/box3d.c3l
./scripts/build-box3d.sh --check
```

This rebuilds box3d and compares every type (and pinned field, where listed) in `scripts/abi-types.txt` against `scripts/abi-sizes.txt`. A drift here means the `$assert` pins in `box3d.c3i` are wrong and calls will corrupt memory. Do not paper over it with `--update` unless the binding is being updated to match in the same change.

## 5. Report honestly

State exactly which steps ran and which passed. Quote the shortest decisive compiler error line for any failure. If a step was skipped — no `test/`, empty `linked-libs/`, `c3c` not matching the 0.8.3 target this repo declares — say which step and why. Do not claim the bindings are verified when only step 1 ran.
