# PolyAI

PolyAI is a AI model designed to play the game PolyTrack.

## Roadmap

- 🟡 PolyTrack simulation environment in C++
- 🔴 Optimized C++ simulation environment
- 🔴 Basic PPO model
- 🔴 Distributed training network
- 🔴 Advanced training and hyper-parameter tuning
- TBD...

## Building

Windows is not supported due to WasmTime not being available on Windows. Please use WSL.

Pre-requisites:
- [WASI SDK](https://github.com/WebAssembly/wasi-sdk)
- [Wasm Time](https://github.com/bytecodealliance/wasmtime)

1. Clone the repository with submodules:

    ```bash
    git clone --recurse-submodules https://github.com/SpideyZac/polyai.git
    cd polyai
    ```

2. Create a build directory:

    ```bash
    cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=/path/to/wasi-sdk/share/cmake/wasi-sdk.cmake -DCMAKE_BUILD_TYPE=Release
    ```

3. Build the project using CMake:

    ```bash
    cmake --build build
    ```

4. Run the executable:

    ```bash
    wasmtime build/polyai
    ```

## Formatting

This project uses `clang-format` for code formatting. To format the code, run:

```bash
./scripts/format.sh
```

## License

PolyAI itself is licensed under the MIT License. See the `LICENSE` file for details. `ammo.js` & `Bullet Physics` are licensed under their own respective licenses.
See the `ammo.js/LICENSE` and `ammo.js/bullet/LICENSE` files for details.
