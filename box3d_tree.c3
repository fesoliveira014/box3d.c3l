module b3;

<*
 Inserts a proxy holding the box and the user data, and returns the id that names it.

 box3d's own check on the box is an assertion this build compiles out, and an invalid one is not
 inert: an inverted box is never reported by any query, an infinite one is reported by all of
 them, and a NaN one spreads into every box above it. A zero box is valid. Ids carry no
 generation counter, so a destroyed proxy's id goes to the next proxy created.

 @return "the id, or INVALID_DEFINITION when the box is inverted or not finite"
*>
fn int? DynamicTree.create_proxy(&self, Aabb aabb, ulong category_bits, ulong user_data) {
    if (!is_valid_aabb(aabb)) return INVALID_DEFINITION~;
    return create_proxy_raw(self, aabb, category_bits, user_data);
}

<*
 Removes the proxy and returns its slot to the free list, from which the next create_proxy() may
 hand it straight back out. box3d asserts the id and this build compiles the assertion out, so a
 repeated destroy would take one slot twice and give one node to two proxies.

 @return "INDEX_OUT_OF_RANGE when the id names no node, or NOT_A_PROXY when no live proxy holds it"
*>
fn void? DynamicTree.destroy_proxy(&self, int proxy_id) {
    check_proxy(self, proxy_id)!;
    destroy_proxy_raw(self, proxy_id);
}

<*
 Removes the proxy and reinserts it under a new box. box3d asserts both the id and the box here
 and this build compiles both out; see create_proxy() for what an invalid box does to a tree.

 @return "INDEX_OUT_OF_RANGE when the id names no node, NOT_A_PROXY when no live proxy holds it, or INVALID_DEFINITION when the box is inverted or not finite"
*>
fn void? DynamicTree.move_proxy(&self, int proxy_id, Aabb aabb) {
    check_proxy(self, proxy_id)!;
    if (!is_valid_aabb(aabb)) return INVALID_DEFINITION~;
    move_proxy_raw(self, proxy_id, aabb);
}

<*
 Widens the proxy's box and every ancestor box that does not already contain it. box3d only ever
 grows an ancestor, so a box the proxy's own already contains shrinks the proxy alone and leaves
 the tree coarser than it needs to be. Guarded as move_proxy() is.

 @return "INDEX_OUT_OF_RANGE when the id names no node, NOT_A_PROXY when no live proxy holds it, or INVALID_DEFINITION when the box is inverted or not finite"
*>
fn void? DynamicTree.enlarge_proxy(&self, int proxy_id, Aabb aabb) {
    check_proxy(self, proxy_id)!;
    if (!is_valid_aabb(aabb)) return INVALID_DEFINITION~;
    enlarge_proxy_raw(self, proxy_id, aabb);
}

<*
 Sets the proxy's side of a query mask and recomputes it up the ancestors. box3d asserts the id
 is a live leaf and compiles that out, and an internal node's bits are the union of its children,
 so a wrong id quietly changes what a filtered query reports.

 @return "INDEX_OUT_OF_RANGE when the id names no node, or NOT_A_PROXY when no live proxy holds it"
*>
fn void? DynamicTree.set_category_bits(&self, int proxy_id, ulong category_bits) {
    check_proxy(self, proxy_id)!;
    set_category_bits_raw(self, proxy_id, category_bits);
}

<*
 The proxy's side of a query mask. box3d checks only the id's range here, and an internal node
 reports the union of its children while a freed slot reports the bits of the proxy that left it,
 so this one is guarded as user_data() is.

 @return "the category bits, INDEX_OUT_OF_RANGE when the id names no node, or NOT_A_PROXY when no live proxy holds it"
*>
fn ulong? DynamicTree.category_bits(&self, int proxy_id) {
    check_proxy(self, proxy_id)!;
    return category_bits_raw(self, proxy_id);
}

<*
 The eight bytes of user data the proxy was created with. box3d's reader is inline in the header
 rather than exported and bounds the id with nothing at all, so this one is the binding's own.

 @return "the user data, INDEX_OUT_OF_RANGE when the id names no node, or NOT_A_PROXY when no live proxy holds it"
*>
fn ulong? DynamicTree.user_data(&self, int proxy_id) {
    check_proxy(self, proxy_id)!;
    return self.nodes[proxy_id].user_data;
}

<*
 The box the tree holds for the proxy. The binding's own reader, and guarded, for the reason
 user_data() gives.

 @return "the box, INDEX_OUT_OF_RANGE when the id names no node, or NOT_A_PROXY when no live proxy holds it"
*>
fn Aabb? DynamicTree.aabb(&self, int proxy_id) {
    check_proxy(self, proxy_id)!;
    return self.nodes[proxy_id].aabb;
}

<*
 The range and leaf checks box3d compiles out of every call that indexes the node array on an id
 the caller supplies. The leaf bit is exact for a live proxy but says nothing about which one: an
 id whose slot has since been handed to a new proxy passes.

 @return "INDEX_OUT_OF_RANGE when the id is outside 0 to node_capacity - 1, or NOT_A_PROXY when no live proxy holds that node"
*>
fn void? check_proxy(DynamicTree* tree, int proxy_id) @private {
    if (proxy_id < 0 || proxy_id >= tree.node_capacity) return INDEX_OUT_OF_RANGE~;
    if ((tree.nodes[proxy_id].flags & TREE_NODE_LEAF) == 0) return NOT_A_PROXY~;
}
