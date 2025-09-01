use std::fs;
use std::path::{Path, PathBuf};

fn read_dir(path: &Path) -> Vec<PathBuf> {
    let mut cpp_files = Vec::new();

    for entry in fs::read_dir(path).expect("Failed to read directory") {
        let entry = entry.expect("Failed to get directory entry");
        let path = entry.path();

        if path.is_dir() {
            if path.display().to_string().contains("MiniCL") {
                continue;
            }
            cpp_files.extend(read_dir(&path));
        } else if path.extension().map_or(false, |ext| ext == "cpp") {
            cpp_files.push(path);
        }
    }

    cpp_files
}

fn main() {
    let include_path_1 = PathBuf::from("ammo.js/bullet/src");
    let include_path_2 = PathBuf::from("ammo.js");

    let mut b = autocxx_build::Builder::new("src/main.rs", &[&include_path_1, &include_path_2])
        .extra_clang_args(&[
            "-std=c++14",
            "-DBT_NO_SIMD_OPERATOR_OVERLOADS",
        ])
        .build().expect("Failed to create builder");

    let cpp_dir = Path::new("ammo.js/bullet/src/");
    let cpp_files = read_dir(cpp_dir);
    b.files(cpp_files);

    b.std("c++14")
        .compile("ammo");

    println!("cargo:rerun-if-changed={}", include_path_2.display());
    println!("cargo:rerun-if-changed=src/main.rs");
}
