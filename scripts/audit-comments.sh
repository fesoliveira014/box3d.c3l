#!/bin/sh
# The comment gate. Five readings, because a per-comment cap cannot see an aggregate:
#
#   1. prose body lines over the cap    -- must be empty
#   2. docstring lines over the width   -- must be empty
#   3. inline // runs over three lines  -- a standing backlog; must never grow
#   4. prose body sitting AT the cap    -- a standing backlog; must never grow
#   5. docstrings over the line budget  -- a standing backlog; must never grow
#
# Prose stops at the first @-directive, which is where C3's own doc parser stops reading free text,
# so a directive and its wrapped continuation lines are not prose. That is why (2) and (5) exist:
# without them a contract moved out of prose and into a @return would leave the cap untouched, and
# a 400-character @return on one physical line would read as a single line. (2) prices the width,
# (5) prices the whole docstring, and between them a directive is no cheaper than the prose it
# replaced.
#
# (4) exists because a ceiling becomes a budget. A cap of six with most docstrings at six is a
# worse outcome than no cap, and only the aggregate shows it: the natural shape of the length
# distribution declines, so a pile-up at the ceiling means length was chosen by the limit rather
# than by the contract.
set -eu
CAP="${1:-6}"
RUN="${2:-3}"
WIDTH="${3:-100}"
BUDGET="${4:-10}"
cd "$(dirname "$0")/.."

FILES=$(ls src/*.c3i src/*.c3 test/tests/*.c3 2>/dev/null || true)

# Emits "file:line:prose:total" for every docstring in $1.
scan() {
    awk -v file="$1" '
        /^[[:space:]]*<\*/ { inside = 1; start = NR; prose = 0; total = 0; directives = 0; next }
        inside && /\*>/ {
            printf "%s:%d:%d:%d\n", file, start, prose, total
            inside = 0; next
        }
        inside {
            line = $0; sub(/^[[:space:]]*/, "", line)
            if (line == "") next
            total += 1
            if (line ~ /^@/) { directives = 1; next }
            if (directives) next
            prose += 1
        }
    ' "$1"
}

echo "== prose body over $CAP lines (must be empty) =="
for f in $FILES; do
    scan "$f" | awk -F: -v cap="$CAP" '$3 > cap { printf "%s:%s: prose body is %s lines\n", $1, $2, $3 }'
done

echo "== docstring lines over $WIDTH characters (must be empty) =="
for f in $FILES; do
    awk -v file="$f" -v width="$WIDTH" '
        /^[[:space:]]*<\*/ { inside = 1; next }
        inside && /\*>/ { inside = 0; next }
        inside { if (length($0) > width) printf "%s:%d: %d characters\n", file, NR, length($0) }
    ' "$f"
done

echo "== inline // runs over $RUN lines =="
for f in $FILES; do
    awk -v file="$f" -v cap="$RUN" '
        /^[[:space:]]*\/\// { if (run == 0) start = NR; run += 1; next }
        { if (run > cap) printf "%s:%d: %d consecutive comment lines\n", file, start, run; run = 0 }
        END { if (run > cap) printf "%s:%d: %d consecutive comment lines\n", file, start, run }
    ' "$f"
done

echo "== prose body sitting exactly at the $CAP-line cap =="
for f in $FILES; do
    scan "$f" | awk -F: -v cap="$CAP" '$3 == cap { printf "%s:%s\n", $1, $2 }'
done

echo "== docstrings over the $BUDGET-line budget, prose and directives together =="
for f in $FILES; do
    scan "$f" | awk -F: -v budget="$BUDGET" '$4 > budget { printf "%s:%s: %s lines\n", $1, $2, $4 }'
done
