module b3;

const uint HASH_INIT = 5381;

<*
 The djb2 hash box3d uses for determinism checks.
*>
fn uint hash(char[] data, uint seed = HASH_INIT) {
    return hash_raw(seed, data.ptr, (int)data.len);
}

<*
 Confirms the linked library uses the float ABI these bindings are declared against. A library
 built with BOX3D_DOUBLE_PRECISION widens every position and silently corrupts every call.

 @return "void, or DOUBLE_PRECISION_LIBRARY when the linked library uses double precision"
*>
fn void? require_float_abi() {
    if (is_double_precision()) return DOUBLE_PRECISION_LIBRARY~;
}

<*
 The colour box3d assigns a constraint graph slot, for a renderer drawing the solver's colouring.
 The last slot is the overflow colour.

 @return "the colour, or INDEX_OUT_OF_RANGE outside 0 to GRAPH_COLOR_COUNT - 1 — box3d asserts the range, this release build compiles that out, and the read past the table answers neither a colour nor the same value from one build to the next"
*>
fn HexColor? graph_color(int index) {
    if (index < 0 || index >= GRAPH_COLOR_COUNT) return INDEX_OUT_OF_RANGE~;
    return graph_color_raw(index);
}
