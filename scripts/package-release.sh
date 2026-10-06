#!/bin/sh
# Builds the distributable packages: one packed .c3l per platform, each a zip whose root holds
# manifest.json. The names are b3-v<version>-<platform>.c3l; c3c resolves a library by the
# manifest's `provides`, not by file name.
#
# What ships is what a consumer compiles and links against, and nothing else: the sources, the
# static library of that platform, the licences and a consumer README. Not the vendored
# submodule, not the build and audit scripts, not the test project.
set -eu

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VERSION="${1:-}"
DIST="$ROOT/dist"
STAGE_ROOT="$DIST/.stage"

[ -n "$VERSION" ] || { echo "usage: scripts/package-release.sh <version>" >&2; exit 1; }

# An untracked file counts: src/ is copied from the working tree, so a source never committed
# would ship. A dirty submodule working tree does not -- vendor/ is not in the artifact -- but a
# submodule at a commit other than the recorded one is, since it is what libbox3d.a was built from.
dirty="$(git -C "$ROOT" status --porcelain --ignore-submodules=dirty)"
if [ -n "$dirty" ]; then
    echo "ERROR: the working tree is dirty. A release is cut from a committed state:" >&2
    printf '%s\n' "$dirty" >&2
    exit 1
fi

if [ ! -f "$ROOT/vendor/box3d/CMakeLists.txt" ]; then
    echo "ERROR: vendor/box3d is empty. Run: git submodule update --init" >&2
    exit 1
fi

# A shipped archive whose layout no longer matches the $assert pins compiled beside it would
# corrupt every call it is used for, so this is a gate rather than a convenience.
"$ROOT/scripts/build-box3d.sh" --check

# Every target the manifest declares must have its library present. A release missing one would
# resolve for a consumer on that platform and then fail at link, which is a worse failure than
# refusing here: c3c reports the manifest's target as supported whether the archive is there or not.
LINUX_LIB="$ROOT/linked-libs/linux-x64/libbox3d.a"
WINDOWS_LIB="$ROOT/linked-libs/windows-x64/box3d.lib"
for lib in "$LINUX_LIB" "$WINDOWS_LIB"; do
    [ -f "$lib" ] || { echo "ERROR: $lib is missing; the release declares the target it belongs to." >&2; exit 1; }
done

rm -rf "$STAGE_ROOT" "$DIST"/b3-v*.c3l "$DIST/SHA256SUMS"
mkdir -p "$DIST"

BOX3D_COMMIT="$(git -C "$ROOT/vendor/box3d" rev-parse --short HEAD)"
BOX3D_DESCRIBE="$(git -C "$ROOT/vendor/box3d" describe --tags 2>/dev/null || echo "$BOX3D_COMMIT")"

pack_platform() {
    platform="$1"
    library="$2"
    stage="$STAGE_ROOT/$platform"
    archive="$DIST/b3-v$VERSION-$platform.c3l"

    mkdir -p "$stage/src" "$stage/linked-libs/$platform"
    cp "$ROOT"/src/*.c3 "$ROOT"/src/*.c3i "$stage/src/"
    cp "$library" "$stage/linked-libs/$platform/"
    cp "$ROOT/LICENSE" "$ROOT/LICENSE.box3d.mit" "$ROOT/NOTICE" "$stage/"

    sed -e "s/\"version\": \"[^\"]*\"/\"version\": \"$VERSION\"/" \
        -e "s/\"box3d-commit\": \"[^\"]*\"/\"box3d-commit\": \"$BOX3D_COMMIT\"/" \
        -e "s/\"box3d-describe\": \"[^\"]*\"/\"box3d-describe\": \"$BOX3D_DESCRIBE\"/" \
        "$ROOT/manifest.json" > "$stage/manifest.json"

    sed -e "s/@VERSION@/$VERSION/g" \
        -e "s/@BOX3D_DESCRIBE@/$BOX3D_DESCRIBE/g" \
        "$ROOT/scripts/README.dist.md" > "$stage/README.md"

    # manifest.json must sit at the archive root: c3c reports "Missing manifest" for an archive
    # that carries it under a directory prefix. Entries are sorted so the archive is reproducible.
    ( cd "$stage" && find . -type f | sed 's|^\./||' | LC_ALL=C sort | zip -q -X "$archive" -@ )
    echo "Wrote $archive ($(wc -c < "$archive") bytes)"
}

pack_platform linux-x64 "$LINUX_LIB"
pack_platform windows-x64 "$WINDOWS_LIB"
rm -rf "$STAGE_ROOT"

( cd "$DIST" && sha256sum b3-v"$VERSION"-linux-x64.c3l b3-v"$VERSION"-windows-x64.c3l > SHA256SUMS )

echo "box3d $BOX3D_DESCRIBE, version $VERSION"
