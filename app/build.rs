fn main() {
    // ── macOS: embed Info.plist so CoreBluetooth grants Bluetooth access ──────
    //
    // On macOS, CBCentralManager silently refuses to scan (state stays
    // "unauthorised") unless the running binary has an embedded Info.plist
    // containing NSBluetoothAlwaysUsageDescription.
    //
    // The standard trick for CLI tools is to stick the plist into the
    //   __TEXT,__info_plist
    // section of the Mach-O binary via the linker `-sectcreate` flag.
    // macOS reads that section exactly as it would an App Bundle's Info.plist.
    //
    // Note: `CARGO_CFG_TARGET_OS` reflects the *target* (not the host),
    // so cross-compilation from Linux → macOS is handled correctly too.
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos") {
        let dir = std::env::var("CARGO_MANIFEST_DIR")
            .expect("CARGO_MANIFEST_DIR must be set by Cargo");

        let plist = format!("{dir}/Info.plist");

        // Each `cargo:rustc-link-arg` call appends one argument to the
        // final linker invocation, so these four together produce:
        //   ld … -sectcreate __TEXT __info_plist /path/to/Info.plist …
        println!("cargo:rustc-link-arg=-sectcreate");
        println!("cargo:rustc-link-arg=__TEXT");
        println!("cargo:rustc-link-arg=__info_plist");
        println!("cargo:rustc-link-arg={plist}");

        // Re-run if the plist changes
        println!("cargo:rerun-if-changed=Info.plist");
    }

    // ── ACT C++ library (feature = "act") ────────────────────────────────────
    //
    // Compiles the ACT-lib C++ sources + ALGLIB + our FFI bridge into a static
    // archive that the Rust linker can consume.  Only built when the `act`
    // Cargo feature is active.
    #[cfg(feature = "act")]
    {
        let act_lib = "ACT-lib/actlib";

        // ACT core sources
        let act_sources = [
            format!("{act_lib}/src/ACT.cpp"),
            format!("{act_lib}/src/ACT_CPU.cpp"),
            format!("{act_lib}/src/ACT_Accelerate.cpp"),
            format!("{act_lib}/src/ACT_MLX.cpp"),
        ];

        // ALGLIB sources (skip x86 kernel files on aarch64)
        let alglib_dir = format!("{act_lib}/lib/alglib/alglib-cpp/src");
        let alglib_sources = [
            format!("{alglib_dir}/ap.cpp"),
            format!("{alglib_dir}/alglibinternal.cpp"),
            format!("{alglib_dir}/alglibmisc.cpp"),
            format!("{alglib_dir}/optimization.cpp"),
            format!("{alglib_dir}/linalg.cpp"),
            format!("{alglib_dir}/solvers.cpp"),
            format!("{alglib_dir}/dataanalysis.cpp"),
            format!("{alglib_dir}/interpolation.cpp"),
            format!("{alglib_dir}/specialfunctions.cpp"),
            format!("{alglib_dir}/statistics.cpp"),
            format!("{alglib_dir}/fasttransforms.cpp"),
            format!("{alglib_dir}/integration.cpp"),
            format!("{alglib_dir}/diffequations.cpp"),
        ];

        // FFI bridge
        let bridge_source = "ffi/act_bridge.cpp";

        let mut build = cc::Build::new();
        build
            .cpp(true)
            .std("c++17")
            .opt_level(3)
            .define("USE_MLX", None)
            .define("ACCELERATE_NEW_LAPACK", None)
            .include(format!("{act_lib}/include"))
            .include(format!("{act_lib}/lib"))
            .include("/usr/local/include")
            .warnings(false); // suppress ALGLIB sprintf deprecation warnings

        for src in &act_sources {
            build.file(src);
        }
        for src in &alglib_sources {
            build.file(src);
        }
        build.file(bridge_source);

        build.compile("act_bridge");

        // Link system libraries
        println!("cargo:rustc-link-search=native=/usr/local/lib");
        println!("cargo:rustc-link-lib=static=mlx");
        println!("cargo:rustc-link-lib=framework=Accelerate");
        println!("cargo:rustc-link-lib=framework=Metal");
        println!("cargo:rustc-link-lib=framework=MetalPerformanceShaders");
        println!("cargo:rustc-link-lib=framework=Foundation");
        println!("cargo:rustc-link-lib=framework=QuartzCore");
        println!("cargo:rustc-link-lib=c++");

        // Re-run if bridge files change
        println!("cargo:rerun-if-changed=ffi/act_bridge.h");
        println!("cargo:rerun-if-changed=ffi/act_bridge.cpp");
    }
}
