fn main() {
    let sdk_shared = "sdk/Linux/c&c++/x86_64-linux-gnu/shared";
    println!("cargo:rustc-link-search=native={sdk_shared}");
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/../../{sdk_shared}");
    println!("cargo:rerun-if-changed=build.rs");
}
