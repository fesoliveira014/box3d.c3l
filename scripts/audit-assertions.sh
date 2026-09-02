#!/bin/sh
# Surveys bound declarations whose C body asserts the validity of an argument -- an assertion this
# release build compiles out, so a bad value is accepted in silence -- and whose docstring says
# nothing about it. CLAUDE.md names a compiled-out assertion as the first contract that earns a
# docstring its length, so this is the line that must never be the one a trim cuts.
#
# This is a survey, not a gate: the list is a standing backlog recorded in .dev/notes.md, and it is
# long because it predates the rule. What it must never do is GROW.
#
# The predicate list below is the whole of what this survey can see, so a family missing from it is
# a blind spot rather than a clean result. Position and the two transform predicates were absent
# until a review found a declaration asserting b3IsValidPosition that the survey could not report;
# adding them raised the backlog from 25 to 31 without a single new omission being written.
# b3IsNormalized was added the same way, and the backlog stayed at 31: nothing public asserts it
# yet. Check the list against math_functions.h whenever box3d moves.
set -eu
cd "$(dirname "$0")/.."

awk '
    /^[[:space:]]*<\*/ { doc = ""; inside = 1; next }
    inside && /\*>/ { inside = 0; next }
    inside { doc = doc " " tolower($0); next }
    /@cname\("/ {
        match($0, /@cname\("[^"]+"\)/)
        sym = substr($0, RSTART + 8, RLENGTH - 10)
        # A @private raw extern carries its contract on the public wrapper that guards it, which
        # lives in a .c3 file this sweep does not read.
        if ($0 ~ /@private/) { doc = ""; next }
        print sym "\t" (index(doc, "assert") || index(doc, "compiled out") ? "documented" : "silent")
        doc = ""
    }
' box3d.c3i | while IFS="$(printf '\t')" read -r sym state; do
    [ "$state" = "silent" ] || continue
    # The C body: from the definition line to the closing brace at column 0.
    body=$(awk -v s="$sym" '
        $0 ~ "^[a-zA-Z_].*[^a-zA-Z0-9_]" s "\\(" { on = 1 }
        on { print }
        on && /^}/ { exit }
    ' vendor/box3d/src/*.c 2>/dev/null || true)
    # Only argument-validity assertions: those a caller trips by passing a bad value, which this
    # build accepts in silence. Internal state assertions are box3d's business, not the caller's.
    if printf '%s\n' "$body" | grep -qE 'B3_ASSERT\([^)]*b3Is(Valid(Float|Vec3|Quat|AABB|Plane|Ray|Position|WorldTransform|Transform)|Normalized)'; then
        echo "$sym: asserts its argument, docstring is silent"
    fi
done
