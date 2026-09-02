#!/bin/sh
# The comment gate. Three readings, because a per-comment cap cannot see an aggregate:
#
#   1. docstrings over the cap        -- must be empty
#   2. inline // runs over three lines -- a standing backlog; must never grow
#   3. docstrings sitting AT the cap   -- a standing backlog; must never grow
#
# (3) exists because a ceiling becomes a budget. A cap of six with most docstrings at six is a
# worse outcome than no cap, and only the aggregate shows it: the natural shape of the length
# distribution declines, so a pile-up at the ceiling means length was chosen by the limit rather
# than by the contract.
set -eu
CAP="${1:-6}"
RUN="${2:-3}"
cd "$(dirname "$0")/.."

FILES=$(ls *.c3i *.c3 test/tests/*.c3 2>/dev/null || true)

echo "== docstrings over $CAP body lines (must be empty) =="
for f in $FILES; do
    awk -v file="$f" -v cap="$CAP" '
        /^[[:space:]]*<\*/ { inside = 1; start = NR; body = 0; next }
        inside && /\*>/ {
            if (body > cap) printf "%s:%d: docstring body is %d lines\n", file, start, body
            inside = 0; next
        }
        inside {
            line = $0; sub(/^[[:space:]]*/, "", line)
            if (line == "" || line ~ /^@(param|return)/) next
            body += 1
        }
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

echo "== docstrings sitting exactly at the $CAP-line cap =="
for f in $FILES; do
    awk -v file="$f" -v cap="$CAP" '
        /^[[:space:]]*<\*/ { inside = 1; start = NR; body = 0; next }
        inside && /\*>/ {
            if (body == cap) printf "%s:%d\n", file, start
            inside = 0; next
        }
        inside {
            line = $0; sub(/^[[:space:]]*/, "", line)
            if (line == "" || line ~ /^@(param|return)/) next
            body += 1
        }
    ' "$f"
done
