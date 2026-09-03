module b3;

<*
 The colour box3d assigns a constraint graph slot, for a renderer drawing the solver's colouring.
 The last slot is the overflow colour. box3d asserts the range and this release build compiles
 that out, so the read past the table answers neither a colour nor a stable value.

 @return `the colour`
 @return? INDEX_OUT_OF_RANGE : `index is outside 0 to GRAPH_COLOR_COUNT - 1`
*>
fn HexColor? graph_color(int index) {
    if (index < 0 || index >= GRAPH_COLOR_COUNT) return INDEX_OUT_OF_RANGE~;
    return graph_color_raw(index);
}
