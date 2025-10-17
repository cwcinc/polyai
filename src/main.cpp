#include "main.h"

int main() {
    using clock = std::chrono::high_resolution_clock;

    auto start = clock::now();

    bool success = DeterminismCheck::determinismCheck();

    auto end = clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;

    std::cout << "Simulation took " << elapsed.count() << " ms\n";

    if (success) {
        std::cout << "Determinism check passed!\n";
    } else {
        std::cout << "Determinism check failed!\n";
        return 1;
    }
}
