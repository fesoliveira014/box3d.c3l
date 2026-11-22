#!/bin/sh
# Runs the box3d callbacks and the origin shift under ThreadSanitizer, Linux only.
#
# Builds the patched copy scripts/build-box3d.sh prepared (build/box3d-src) with clang and
# -fsanitize=thread, links scripts/tsan_harness.c against it, and runs a 256-box pile on four
# workers: without and with the contact material callback, without and with an origin shift every
# 30 steps, and once with a deliberate race in the callback, which the sanitizer must report.
# Fails when a clean mode reports anything or the deliberate race goes unreported.
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SOURCE="$ROOT/build/box3d-src"
OUT="$SOURCE/build-tsan"
[ -f "$SOURCE/.patched" ] || { echo "ERROR: run scripts/build-box3d.sh first" >&2; exit 1; }

cmake -S "$SOURCE" -B "$OUT" -DCMAKE_C_COMPILER=clang -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_C_FLAGS="-fsanitize=thread" -DBUILD_SHARED_LIBS=OFF -DBOX3D_DOUBLE_PRECISION=OFF \
    -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=OFF -DBOX3D_BENCHMARKS=OFF -DBOX3D_DOCS=OFF \
    -DBOX3D_VALIDATE=ON >/dev/null
cmake --build "$OUT" --target box3d --parallel >/dev/null
LIB="$(find "$OUT" -name libbox3d.a -print -quit)"
clang -std=c17 -fsanitize=thread -g -O1 -I"$SOURCE/include" "$ROOT/scripts/tsan_harness.c" "$LIB" \
    -lm -lpthread -o "$OUT/harness"

status=0
for mode in "0 0" "1 0" "0 1" "1 1" "1 1 1"; do
    log="$OUT/tsan_$(echo "$mode" | tr ' ' _).log"
    # Without address randomization: on a kernel with high mmap entropy the sanitizer's runtime
    # aborts at startup with "unexpected memory mapping".
    TSAN_OPTIONS="halt_on_error=0" setarch "$(uname -m)" -R "$OUT/harness" $mode 2> "$log" || status=1
    warnings="$(grep -c 'WARNING: ThreadSanitizer' "$log" || true)"
    echo "callback, shift, race = $mode: $warnings sanitizer reports"
    if grep -q FATAL "$log"; then echo "ERROR: the sanitizer did not start; see $log" >&2; status=1; fi
    case "$mode" in
        "1 1 1") [ "$warnings" -gt 0 ] || { echo "ERROR: the deliberate race went unreported" >&2; status=1; } ;;
        *)       [ "$warnings" -eq 0 ] || { echo "ERROR: see $log" >&2; status=1; } ;;
    esac
done
exit $status
