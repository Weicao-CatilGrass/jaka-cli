use std::env;
use std::path::{Path, PathBuf};

fn main() {
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();

    match target_os.as_str() {
        "linux" => {
            let static_lib = "sdk/Linux/c&c++/x86_64-linux-gnu/static";
            let shared_lib = "sdk/Linux/c&c++/x86_64-linux-gnu/shared";
            // Prefer the static lib so the result is a single binary. Fall back
            // to the shared lib with a relative rpath when the static lib is absent
            if Path::new(static_lib).join("libjakaAPI.a").exists() {
                println!("cargo:rustc-link-search=native={static_lib}");
                println!("cargo:rustc-link-lib=static=jakaAPI");
                // The SDK is C++ code, so pull in the C++ runtime
                println!("cargo:rustc-link-lib=dylib=stdc++");
                println!("cargo:rustc-link-lib=m");
            } else {
                println!("cargo:rustc-link-search=native={shared_lib}");
                // Runtime lookup via a relative rpath: the binary lives in
                // target/<profile>/, the SDK is ../../sdk/... from there
                println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/../../{shared_lib}");
            }
        }
        "windows" => {
            let sdk_x64 = "sdk/Windows/c&c++/x64";
            println!("cargo:rustc-link-search=native={sdk_x64}");
            // The Windows loader only finds jakaAPI.dll next to the exe or on
            // PATH, so copy it to the build profile directory
            copy_dll(Path::new(sdk_x64).join("jakaAPI.dll"));
            // The gnu toolchain searches for libjakaAPI.a, but the SDK only
            // ships an MSVC import library, so link it by exact file name
            if env::var("CARGO_CFG_TARGET_ENV").unwrap_or_default() == "gnu" {
                println!("cargo:rustc-link-arg=-l:jakaAPI.lib");
                // Link the C++ runtime statically so the exe does not need
                // libstdc++-6.dll or libgcc_s_seh-1.dll on the target machine.
                // The libraries go at the end of the link line, after the shim
                // objects, because static libraries resolve symbols left to right
                println!("cargo:rustc-link-arg=-Wl,-Bstatic");
                // Group the static libraries so circular references between
                // the C++ runtime, exception support, pthread and the C
                // runtime resolve
                println!("cargo:rustc-link-arg=-Wl,--start-group");
                println!("cargo:rustc-link-arg=-lstdc++");
                println!("cargo:rustc-link-arg=-lgcc_eh");
                println!("cargo:rustc-link-arg=-lgcc");
                println!("cargo:rustc-link-arg=-l:libpthread.a");
                println!("cargo:rustc-link-arg=-lmsvcrt");
                println!("cargo:rustc-link-arg=-lmingwex");
                println!("cargo:rustc-link-arg=-Wl,--end-group");
                println!("cargo:rustc-link-arg=-Wl,-Bdynamic");
                // winpthread calls Windows APIs, resolve them again after the
                // static group. Re-linking system import libs is harmless
                println!("cargo:rustc-link-arg=-lkernel32");
                println!("cargo:rustc-link-arg=-lntdll");
                println!("cargo:rustc-link-arg=-luser32");
                println!("cargo:rustc-link-arg=-luserenv");
                println!("cargo:rustc-link-arg=-ladvapi32");
            }
        }
        other => panic!("unsupported target OS: {other}"),
    }

    // The exception-safe C++ shim around the SDK
    cc::Build::new()
        .cpp(true)
        // Do not let cc add the C++ runtime automatically, each platform
        // links it explicitly with the desired variant
        .cpp_link_stdlib(None)
        .file("ffi/sdk_shim.cpp")
        .compile("jaka_shim");

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=ffi/sdk_shim.cpp");
}

/// Copy the SDK dll next to the built executable. build.rs runs on the host,
/// so this must stay platform independent and only be called for Windows targets
fn copy_dll(src: PathBuf) {
    let out_dir = env::var("OUT_DIR").expect("OUT_DIR not set");
    // OUT_DIR is target/<profile>/build/<pkg>-<hash>/out, so the profile
    // directory is the third ancestor
    let profile_dir = Path::new(&out_dir)
        .ancestors()
        .nth(3)
        .expect("cannot locate the profile directory from OUT_DIR");
    let dst = profile_dir.join("jakaAPI.dll");
    std::fs::copy(&src, &dst)
        .unwrap_or_else(|e| panic!("failed to copy {} to {}: {e}", src.display(), dst.display()));
}
