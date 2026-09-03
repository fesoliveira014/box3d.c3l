module b3;

<*
 Inserts a proxy holding the box and the user data. A destroyed proxy's id goes to the next one.

 box3d's check on the box is compiled out here and an invalid box is not inert: inverted, no query
 reports it; infinite, every query does; NaN, the boxes above it corrupt — box3d's float min and
 max return whichever operand is not the NaN, so an ancestor swallows a real bound and proxies
 fall out of a root box that still reads clean. A zero box is valid.

 @return `the id`
 @return? INVALID_DEFINITION : `the box is inverted or not finite`
*>
fn int? DynamicTree.create_proxy(&self, Aabb aabb, ulong category_bits, ulong user_data) {
    if (!is_valid_aabb(aabb)) return INVALID_DEFINITION~;
    return create_proxy_raw(self, aabb, category_bits, user_data);
}

<*
 Removes the proxy and returns its slot to the free list, from which the next create_proxy() may
 hand it straight back out. box3d asserts the id and this build compiles the assertion out, so a
 repeated destroy would take one slot twice and give one node to two proxies.

 @return? check_proxy!
*>
fn void? DynamicTree.destroy_proxy(&self, int proxy_id) {
    check_proxy(self, proxy_id)!;
    destroy_proxy_raw(self, proxy_id);
}

<*
 Removes the proxy and reinserts it under a new box. box3d asserts both the id and the box here
 and this build compiles both out; create_proxy() names which boxes are refused and why.

 @return? check_proxy!
 @return? INVALID_DEFINITION : `the box is inverted or not finite`
*>
fn void? DynamicTree.move_proxy(&self, int proxy_id, Aabb aabb) {
    check_proxy(self, proxy_id)!;
    if (!is_valid_aabb(aabb)) return INVALID_DEFINITION~;
    move_proxy_raw(self, proxy_id, aabb);
}

<*
 Widens the proxy's box and every ancestor box that does not already contain it, and takes the
 same guards move_proxy() does. It widens on a strict comparison, so a box the proxy already
 contains changes no ancestor — and neither does a NaN one, unlike through the other two calls,
 though a later rebuild still spreads it. Any call marks every ancestor enlarged even when it
 moved no box, so the next partial rebuild() sorts them all.

 @return? check_proxy!
 @return? INVALID_DEFINITION : `the box is inverted or not finite`
*>
fn void? DynamicTree.enlarge_proxy(&self, int proxy_id, Aabb aabb) {
    check_proxy(self, proxy_id)!;
    if (!is_valid_aabb(aabb)) return INVALID_DEFINITION~;
    enlarge_proxy_raw(self, proxy_id, aabb);
}

<*
 Sets the proxy's side of a query mask and recomputes it up the ancestors. box3d never range
 checks this id at all and asserts only the leaf, compiled out here; an internal node's bits are
 the union of its children, so setting the root's to 0 takes every proxy out of every query.

 @return? check_proxy!
*>
fn void? DynamicTree.set_category_bits(&self, int proxy_id, ulong category_bits) {
    check_proxy(self, proxy_id)!;
    set_category_bits_raw(self, proxy_id, category_bits);
}

<*
 The proxy's side of a query mask. box3d checks only the id's range here, and an internal node
 reports the union of its children while a freed slot reports the bits of the proxy that left it,
 so this one is guarded as user_data() is.

 @return `the category bits`
 @return? check_proxy!
*>
fn ulong? DynamicTree.category_bits(&self, int proxy_id) {
    check_proxy(self, proxy_id)!;
    return category_bits_raw(self, proxy_id);
}

<*
 The eight bytes of user data the proxy was created with. box3d's reader is inline in the header
 rather than exported and bounds the id with nothing at all, so this one is the binding's own.

 @return `the user data`
 @return? check_proxy!
*>
fn ulong? DynamicTree.user_data(&self, int proxy_id) {
    check_proxy(self, proxy_id)!;
    return self.nodes[proxy_id].user_data;
}

<*
 The box the tree holds for the proxy. The binding's own reader, and guarded, for the reason
 user_data() gives.

 @return `the box`
 @return? check_proxy!
*>
fn Aabb? DynamicTree.aabb(&self, int proxy_id) {
    check_proxy(self, proxy_id)!;
    return self.nodes[proxy_id].aabb;
}

<*
 Offers fcn each proxy the search bound still admits. The order is not a ranking: the tree
 measures distance to node boxes only, to prune and to order siblings, so a nearer proxy is
 routinely offered after a farther one. See TreeClosestFn for what fcn's return does, and query()
 for the mask flags and for the traversal stack's limit, which applies here too.

 @param search_bound : `the squared distance to search within; a large one reaches every proxy,
   and 0 admits none of them and reports no error`
*>
fn ClosestSearch DynamicTree.query_closest(
  &self,
  Vec3 point,
  ulong mask_bits,
  bool require_all_bits,
  TreeClosestFn fcn,
  void* context,
  float search_bound,
) {
    TreeStats stats = query_closest_raw(
      tree: self,
      point: point,
      mask_bits: mask_bits,
      require_all_bits: require_all_bits,
      fcn: fcn,
      context: context,
      search_bound: &search_bound,
    );
    return { .stats = stats, .search_bound = search_bound };
}

<*
 Reads back a tree save() wrote, multiplying every node box by scale as it goes. The caller owns
 it and must destroy() it exactly once. box3d zeroes the struct for a path it cannot open, an
 unknown version and a node array that falls short, but not for a file cut inside the header,
 where the bytes that arrived stand over zeroes. It allocates the nodes only after reading that
 header whole and save() writes a null in their place, so every refusal arrives with none.

 @param scale : `what each node box's bounds are multiplied by, 1 reproducing the saved tree`
 @return `the tree`
 @return? FILE_UNAVAILABLE : `the path could not be read, or holds no tree this build knows`
*>
fn DynamicTree? load_tree(ZString path, float scale) {
    DynamicTree tree = load_tree_raw(path, scale);
    if (tree.nodes == null) return FILE_UNAVAILABLE~;
    return tree;
}
