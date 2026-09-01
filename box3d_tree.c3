module b3;

<*
 Inserts a proxy holding the box and the user data, and returns the id that names it.

 box3d's own check on the box is an assertion this build compiles out, and an invalid one is not
 inert: an inverted box is never reported by any query, an infinite one is reported by all of them,
 and a NaN one corrupts the boxes above it, hiding unrelated proxies until a full rebuild. A zero
 box is valid. category_bits is the proxy's side of a query's mask. Ids carry no generation
 counter, so a destroyed proxy's id goes to the next proxy created.

 @return "the id, or INVALID_DEFINITION when the box is inverted or not finite"
*>
fn int? DynamicTree.create_proxy(&self, Aabb aabb, ulong category_bits, ulong user_data) {
    if (!is_valid_aabb(aabb)) return INVALID_DEFINITION~;
    return create_proxy_raw(self, aabb, category_bits, user_data);
}

<*
 The eight bytes of user data the proxy was created with.

 box3d's reader is inline in the header rather than exported and indexes the node array unbounded,
 so this one is the binding's own and restores both of the checks box3d compiles out: the index,
 and that the node is a live leaf rather than an internal node or a freed slot, either of which
 reads as a plausible number. An id whose slot has since been reused is a leaf and reads the new
 proxy's data — that one no guard can catch.

 @return "the user data, INDEX_OUT_OF_RANGE when the id is outside 0 to node_capacity - 1, or NOT_A_PROXY when no live proxy holds that node"
*>
fn ulong? DynamicTree.user_data(&self, int proxy_id) {
    if (proxy_id < 0 || proxy_id >= self.node_capacity) return INDEX_OUT_OF_RANGE~;
    if ((self.nodes[proxy_id].flags & TREE_NODE_LEAF) == 0) return NOT_A_PROXY~;
    return self.nodes[proxy_id].user_data;
}

<*
 The box the tree holds for the proxy. Guarded as user_data() is, and promising exactly as little
 about a recycled id — see there.

 @return "the box, INDEX_OUT_OF_RANGE when the id is outside 0 to node_capacity - 1, or NOT_A_PROXY when no live proxy holds that node"
*>
fn Aabb? DynamicTree.aabb(&self, int proxy_id) {
    if (proxy_id < 0 || proxy_id >= self.node_capacity) return INDEX_OUT_OF_RANGE~;
    if ((self.nodes[proxy_id].flags & TREE_NODE_LEAF) == 0) return NOT_A_PROXY~;
    return self.nodes[proxy_id].aabb;
}
