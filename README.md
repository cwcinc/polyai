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

This is most likely temporary.

Due to some Windows-specific issues, you must build this project with Windows-subsystem-for-Linux (WSL) or a Linux machine.

1. Clone the repository with submodules:

    ```bash
    git clone --recurse-submodules https://github.com/SpideyZac/polyai.git
    cd polyai
    ```

2. Create a build directory and navigate into it:

    ```bash
    cmake -S . -B build -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
    ```

3. Build the project using CMake:

    ```bash
    cmake --build build --config Release
    ```

4. Run the executable:

    ```bash
    ./build/polyai
    ```

## Formatting

This project uses `clang-format` for code formatting. To format the code, run:

```bash
./scripts/format.sh
```

## License

PolyAI itself is licensed under the MIT License. See the `LICENSE` file for details. `ammo.js` & `Bullet Physics` are licensed under their own respective licenses.
See the `ammo.js/LICENSE` and `ammo.js/bullet/LICENSE` files for details.
