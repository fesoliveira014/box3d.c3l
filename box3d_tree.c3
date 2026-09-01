module b3;

<*
 The eight bytes of user data the proxy was created with.

 box3d's own reader for this is inline in the C header rather than an exported symbol, and it
 indexes the node array with no bound of any kind, so this one is the binding's own and carries
 a guard box3d does not.

 That guard is an index bound and nothing more, because node_capacity is the only bound the tree
 carries. Proxy ids are recycled and hold no generation counter, so an id whose proxy has been
 destroyed is still in range: it reads the slot as it stands now, which is the next proxy created
 there, or an internal node whose eight bytes are two child indices, or what the destroyed proxy
 left behind. See TreeNode for the three a node can be. A destroyed tree has a capacity of 0 and
 refuses every id.

 @return "the user data, or INDEX_OUT_OF_RANGE when the id is outside 0 to node_capacity - 1"
*>
fn ulong? DynamicTree.user_data(&self, int proxy_id) {
    if (proxy_id < 0 || proxy_id >= self.node_capacity) return INDEX_OUT_OF_RANGE~;
    return self.nodes[proxy_id].user_data;
}

<*
 The box the tree holds for the proxy.

 The binding owns this reader and its guard for the reason user_data() gives, and an id in range
 promises exactly as little here — see there.

 @return "the box, or INDEX_OUT_OF_RANGE when the id is outside 0 to node_capacity - 1"
*>
fn Aabb? DynamicTree.aabb(&self, int proxy_id) {
    if (proxy_id < 0 || proxy_id >= self.node_capacity) return INDEX_OUT_OF_RANGE~;
    return self.nodes[proxy_id].aabb;
}
