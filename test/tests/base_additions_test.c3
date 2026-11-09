module b3_test;

import b3;
import std::core::cpudetect;

fn void the_linked_library_was_built_for_the_manifold_size_the_binding_declares() @test {
    b3::Manifold manifold;
    b3::CollisionManifold collision;

    assert(b3::max_manifold_points() == b3::MAX_MANIFOLD_POINTS);
    assert((usz)b3::max_manifold_points() == manifold.points.len,
        "so every Manifold the binding reads has the length box3d writes");
    assert((usz)b3::max_manifold_points() == collision.points.len);
}

// box3d also asks the OS through XGETBV whether it saves the wide registers, which CPUID cannot
// report; every OS this package ships for does, so the CPUID bits are the whole answer here.
fn void avx2_is_reported_exactly_when_the_cpu_advertises_it() @test {
    CpuId leaf0 = cpudetect::x86_cpuid(0);
    bool advertised = false;
    if (leaf0.eax >= 7) {
        CpuId features = cpudetect::x86_cpuid(1);
        CpuId extended = cpudetect::x86_cpuid(7);
        uint osxsave_and_avx = (1u << 27) | (1u << 28);
        advertised = (features.ecx & osxsave_and_avx) == osxsave_and_avx && (extended.ebx & (1u << 5)) != 0;
    }

    assert(b3::is_avx2_available() == advertised);
}
