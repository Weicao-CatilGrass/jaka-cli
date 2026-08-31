use std::env;
use std::path::{Path, PathBuf};

fn main() {
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();

    match target_os.as_str() {
        "linux" => {
            let sdk_shared = "sdk/Linux/c&c++/x86_64-linux-gnu/shared";
            println!("cargo:rustc-link-search=native={sdk_shared}");
            // Runtime lookup via a relative rpath: the binary lives in
            // target/<profile>/, the SDK is ../../sdk/... from there
            println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/../../{sdk_shared}");
        }
        "windows" => {
            let sdk_x64 = "sdk/Windows/c&c++/x64";
            println!("cargo:rustc-link-search=native={sdk_x64}");
            // The Windows loader only finds jakaAPI.dll next to the exe or on
            // PATH, so copy it to the build profile directory
            copy_dll(Path::new(sdk_x64).join("jakaAPI.dll"));
        }
        other => panic!("unsupported target OS: {other}"),
    }

    println!("cargo:rerun-if-changed=build.rs");
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
