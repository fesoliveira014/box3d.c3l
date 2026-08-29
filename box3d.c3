module b3;

const uint HASH_INIT = 5381;

<*
 The djb2 hash box3d uses for determinism checks.
*>
fn uint hash(char[] data, uint seed = HASH_INIT) {
    return hash_bytes(seed, data.ptr, (int)data.len);
}

<*
 Confirms the linked library uses the float ABI these bindings are declared against. A library
 built with BOX3D_DOUBLE_PRECISION widens every position and silently corrupts every call.

 @return "void, or DOUBLE_PRECISION_LIBRARY when the linked library uses double precision"
*>
fn void? assert_float_abi() {
    if (is_double_precision()) return DOUBLE_PRECISION_LIBRARY~;
}
