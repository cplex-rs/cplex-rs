use glob::glob;
use std::env;
use std::path::PathBuf;

fn main() {
    let building_docs = std::env::var("DOCS_RS").is_ok();
    let cplex_include_path = if building_docs {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("include")
            .join("22010000")
    } else {
        let cplex_installation_path = env::var("CPLEX_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            glob("/opt/ibm/ILOG/*/cplex")
                .expect("Invalid glob pattern")
                .filter_map(|path| path.ok())
                .next()
                .expect("No valid CPLEX installation path found. Please set the env variable 'CPLEX_PATH' with the CPLEX installation directory or install CPLEX in the default location.")
        });

        let os = env::consts::OS;
        let arch = std::env::consts::ARCH;
        println!("cargo:warning=Detected OS: {}", os);
        println!("cargo:warning=Detected arch: {}", arch);

        let cplex_include = cplex_installation_path.join("include");

        if os == "linux" && arch == "x86_64" {
            let lib = cplex_installation_path.join("lib/x86-64_linux/static_pic");
            println!("cargo:rustc-link-search={}", lib.display());
            println!("cargo:rustc-link-lib=cplex");
        } else if os == "macos" && arch == "aarch64" {
            let lib = cplex_installation_path.join("lib/arm64_osx/static_pic");
            println!("cargo:rustc-link-search={}", lib.display());
            println!("cargo:rustc-link-lib=cplex");
        } else if os == "windows" && arch == "x86_64" {
            let lib = cplex_installation_path.join("lib/x64_windows_msvc14/stat_mda");
            println!("cargo:rustc-link-search={}", lib.display());
            // CPLEX ships a versioned lib on Windows (e.g. cplex2211.lib).
            // Discover it at build time rather than hard-coding the version.
            let lib_name = std::fs::read_dir(&lib)
                .expect("Could not read CPLEX lib directory")
                .filter_map(|e| e.ok())
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .find(|name| name.starts_with("cplex") && name.ends_with(".lib"))
                .expect("Could not find cplex*.lib in CPLEX lib directory");
            let stem = lib_name.trim_end_matches(".lib");
            println!("cargo:rustc-link-lib={stem}");
        } else {
            panic!("Unsupported OS-arch combination: {}-{}", os, arch);
        };

        cplex_include
    };

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let bindings = bindgen::Builder::default()
        // The input header we would like to generate
        // bindings for.
        .header(
            cplex_include_path
                .join("ilcplex")
                .join("cplex.h")
                .to_string_lossy(),
        )
        .clang_arg(format!(
            "-F{}",
            cplex_include_path.as_os_str().to_string_lossy()
        ))
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .allowlist_item("CPX.*")
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
