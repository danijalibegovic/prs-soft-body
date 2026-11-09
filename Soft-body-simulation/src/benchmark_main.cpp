
#include "physics/SoftBodyAdapter.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr float dt = 1.0f / 60.0f;
constexpr int warmup_steps = 10;
constexpr int measured_steps = 100;

struct Result {
    std::size_t nodes = 0;
    std::size_t faces = 0;
    double total_ms = 0.0;
    double average_ms = 0.0;
    double steps_per_second = 0.0;
};

Result run_benchmark(
    const std::string& shape,
    std::size_t resolution
) {
    SoftWorld* world = softWorldCreate({0.0f, -9.81f, 0.0f});

    if (!world) {
        throw std::runtime_error("World creation failed");
    }

    SoftBody* body = nullptr;

    if (shape == "cloth") {
        body = softBodyCreateCloth(
            world,
            {-2.0f, 5.0f, 0.0f},
            4.0f,
            4.0f,
            resolution,
            resolution,
            1.0f
        );
    } else if (shape == "sphere") {
        body = softBodyCreateSphere(
            world,
            {0.0f, 5.0f, 0.0f},
            1.0f,
            resolution,
            1.0f
        );
    } else {
        softWorldDestroy(world);
        throw std::runtime_error("Unknown shape");
    }

    if (!body) {
        softWorldDestroy(world);
        throw std::runtime_error("Body creation failed");
    }

    Result result;
    result.nodes = softBodyGetNodeCount(body);
    result.faces = softBodyGetFaceCount(body);

    for (int i = 0; i < warmup_steps; ++i) {
        if (!softWorldStep(world, dt)) {
            softWorldDestroy(world);
            throw std::runtime_error("Warmup step failed");
        }
    }

    const auto start = std::chrono::steady_clock::now();

    for (int i = 0; i < measured_steps; ++i) {
        if (!softWorldStep(world, dt)) {
            softWorldDestroy(world);
            throw std::runtime_error("Measured step failed");
        }
    }

    const auto end = std::chrono::steady_clock::now();

    result.total_ms =
        std::chrono::duration<double, std::milli>(
            end - start
        ).count();

    result.average_ms =
        result.total_ms / measured_steps;

    result.steps_per_second =
        1000.0 / result.average_ms;

    softWorldDestroy(world);
    return result;
}

} // namespace

int main() {
    try {
        std::ofstream csv("xpbd_benchmark.csv");

        if (!csv) {
            std::cerr << "Cannot create CSV file\n";
            return 1;
        }

        csv << "engine,shape,resolution,nodes,faces,"
               "warmup_steps,measured_steps,total_ms,"
               "average_ms,steps_per_second\n";

        std::cout << "Serial CPU XPBD benchmark\n";
        std::cout << "-------------------------\n";

        for (const std::string shape : {"cloth", "sphere"}) {
            for (std::size_t resolution : {8, 16, 24}) {
                std::cout
                    << "Running " << shape
                    << " resolution=" << resolution
                    << " ... " << std::flush;

                const Result result =
                    run_benchmark(shape, resolution);

                csv << std::fixed << std::setprecision(6)
                    << "XPBD_SERIAL_CPU,"
                    << shape << ','
                    << resolution << ','
                    << result.nodes << ','
                    << result.faces << ','
                    << warmup_steps << ','
                    << measured_steps << ','
                    << result.total_ms << ','
                    << result.average_ms << ','
                    << result.steps_per_second << '\n';

                std::cout
                    << std::fixed << std::setprecision(4)
                    << result.average_ms
                    << " ms/step\n";
            }
        }

        std::cout << "\nSaved: xpbd_benchmark.csv\n";
        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << "Benchmark error: "
                  << error.what() << '\n';
        return 1;
    }
}
