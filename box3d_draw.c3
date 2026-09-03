module b3;

<*
 The colour box3d assigns a constraint graph slot, for a renderer drawing the solver's colouring.
 The last slot is the overflow colour.

 @return "the colour, or INDEX_OUT_OF_RANGE outside 0 to GRAPH_COLOR_COUNT - 1 — box3d asserts the range, this release build compiles that out, and the read past the table answers neither a colour nor the same value from one build to the next"
*>
fn HexColor? graph_color(int index) {
    if (index < 0 || index >= GRAPH_COLOR_COUNT) return INDEX_OUT_OF_RANGE~;
    return graph_color_raw(index);
}
