# Week 1 Progress: Project Foundation

## Status

The project foundation is complete and working. The repository is configured
as a C++20 CMake project with:

- a reusable `soft_body_core` static library;
- a `soft_body_benchmark` executable;
- a `soft_body_core_tests` test executable;
- CTest integration;
- MinGW Debug and Release CMake presets;
- public headers separated from implementation files;
- generated build files excluded from version control.

The benchmark currently prints its version and exits. This confirms that the
project configures, compiles, links, and runs, but it does not yet perform
particle simulation or performance measurements.

## Required software

Yes, every contributor needs both a C++ compiler and CMake.

### C++ compiler

The project uses C++20. On Windows, contributors can use either:

- MinGW-w64 with `g++`; or
- Microsoft Visual C++ through Visual Studio or Visual Studio Build Tools.

The checked-in presets currently target MinGW-w64 and use the `g++` compiler
with the `MinGW Makefiles` generator. Contributors using MSVC can configure
the project with a Visual Studio generator or add an MSVC-specific preset.

### CMake

CMake generates the native build files used to compile the project. It does
not include a C++ compiler, so installing CMake alone is not sufficient.

The minimum supported CMake version in the project is 3.25. A newer version
is acceptable.

### Git

Git is needed to clone the repository, create branches, pull teammate work,
and submit changes for review. Git is not part of the compilation process,
but it is required for collaborative development.

## Verifying a local installation

From a PowerShell terminal, contributors should verify:

```text
cmake --version
g++ --version
git --version
```

The current MinGW presets require `g++` to be available on `PATH`. If a
contributor uses MSVC instead, the Visual Studio developer environment must
be active or an MSVC-specific CMake preset must be used.

## Getting started

After cloning the repository:

1. Open a terminal at the repository root.
2. Configure the Debug build:

   ```text
   cmake --preset mingw-debug
   ```

3. Build all configured targets:

   ```text
   cmake --build --preset mingw-debug
   ```

4. Run the tests:

   ```text
   ctest --preset mingw-debug --output-on-failure
   ```

5. Run the benchmark scaffold:

   ```text
   .\build\mingw-debug\soft_body_benchmark.exe
   ```

The expected current output is:

```text
soft_body_benchmark 0.1.0
```

The Release workflow uses the corresponding `mingw-release` preset:

```text
cmake --preset mingw-release
cmake --build --preset mingw-release
ctest --preset mingw-release --output-on-failure
```

The `build/` directory is generated output. It should not be committed or
edited manually. If a stale configuration causes problems, the generated
build directory can be removed and configured again.

## Repository structure

```text
Soft-body-simulation/
├── CMakeLists.txt
├── CMakePresets.json
├── README.md
├── .gitignore
├── include/
│   └── soft_body/
│       └── soft_body_core.hpp
├── src/
│   ├── soft_body_core.cpp
│   └── benchmark_main.cpp
├── tests/
│   └── soft_body_core_tests.cpp
├── docs/
│   └── research papers
├── progress/
│   └── week-1.md
└── build/
    └── generated files; ignored by Git
```

### Directory responsibilities

`include/` contains public headers. Anything a separate executable or future
renderer needs to include belongs here.

`src/` contains library implementation files and executable entry points.
Physics implementation belongs in the library; benchmark orchestration belongs
in the benchmark executable or benchmark-specific source files.

`tests/` contains automated correctness tests. Tests should verify behavior
and numerical results, not implementation details.

`docs/` contains academic source material and project-wide technical
documentation.

`progress/` contains chronological project handoffs. Each weekly document
records what works, how to reproduce it, and what should happen next.

`build/` contains only generated files. It is intentionally ignored by Git.

## Contribution workflow

Each teammate should:

1. update their local branch;
2. create a branch for one coherent task;
3. make changes only in the relevant source, header, test, and CMake files;
4. build the Debug preset;
5. run CTest;
6. run the relevant benchmark or executable;
7. inspect `git diff` and `git status`;
8. submit the branch for review.

Changes should not include generated files from `build/`. A task is complete
only when the new behavior is both implemented and tested.

## First implementation stage: Stage 1

The next milestone is the CPU data-level parallelism stage. It should provide
a scalar reference implementation first, then compare alternative memory
layouts and SIMD implementations against that reference.

The Stage 1 result should:

- represent particle data in AoS and SoA layouts;
- implement the reference particle update;
- implement scalar AoS and SoA variants;
- implement compiler-assisted vectorization;
- implement `#pragma omp simd` as single-threaded SIMD;
- implement an explicit AVX2 variant where supported;
- handle particle counts that are not multiples of the SIMD width;
- compare optimized results with the scalar reference;
- measure timing and throughput;
- preserve the existing CMake and test workflow.

The first Stage 1 version should use the simple particle update workload
already defined by the project documentation. Full XPBD mesh constraints are a
later physics workload; they should not be mixed into the initial SIMD kernel
task before the data representation and measurement framework are stable.

## Suggested task split

These tasks can be assigned independently, with the dependencies shown below.

### Task 1: Particle data model

Define the common particle types and the AoS and SoA storage representations.
Establish ownership, sizes, initialization, and conversion between layouts.
Keep the public types independent of OpenGL and avoid embedding benchmark
timing into the data structures.

**Depends on:** none.

### Task 2: Scalar reference kernels

Implement the scalar AoS and scalar SoA particle update. This is the numerical
and correctness reference for every optimized version.

The reference must define boundary behavior, damping, bounce behavior, and
floating-point comparison expectations clearly.

**Depends on:** Task 1.

### Task 3: Correctness tests and fixtures

Create deterministic particle inputs and tests that compare AoS and SoA
results. Include ordinary particles, boundary collisions, empty input, small
input, and a count that is not divisible by eight.

These tests should be usable before the optimized implementations exist.

**Depends on:** Tasks 1 and 2.

### Task 4: Benchmark measurement

Extend the benchmark executable with timing, warm-up, repeated measurements,
throughput, and a result sink or checksum. Keep setup time separate from the
timed kernel.

The benchmark must identify which implementation is being measured and must
not allow the compiler to remove unused work.

**Depends on:** Task 2.

### Task 5: Compiler and OpenMP SIMD variants

Implement the ordinary auto-vectorization variant and the `#pragma omp simd`
variant over the SoA representation. Verify that the OpenMP directive is used
only for SIMD and does not introduce thread-level parallelism.

Compare both variants with the scalar reference and record compiler/toolchain
requirements if OpenMP support differs between MinGW and MSVC.

**Depends on:** Tasks 2 and 3.

### Task 6: Explicit AVX2 variant

Implement the eight-float AVX2 kernel with masked boundary handling and a scalar
remainder path. Provide a safe fallback or clear unsupported-ISA behavior on
machines without AVX2.

The AVX2 implementation must be tested against the scalar reference with a
floating-point tolerance.

**Depends on:** Tasks 2, 3, and 4.

### Task 7: Stage 1 CMake and documentation integration

Register new files in the CMake targets, preserve the Debug and Release
presets, document any compiler flags or optional dependencies, and update the
benchmark instructions with the new modes.

This can be completed incrementally by each implementation owner, but one
person should verify the final integrated build.

**Depends on:** the implementation tasks being integrated.

## Recommended assignment order

The safest order is:

1. Task 1;
2. Task 2;
3. Task 3 and Task 4 in parallel;
4. Task 5 and Task 6 in parallel;
5. Task 7 and final integration.

Tasks 3 and 4 can begin independently once the scalar reference exists.
Tasks 5 and 6 must use the same reference behavior rather than defining their
own physics rules.

## Definition of done for Week 1 foundation

The foundation is complete when a new contributor can:

- install or verify a C++20 compiler, CMake, and Git;
- configure the repository using a documented preset;
- build the library, benchmark, and tests;
- run the smoke test successfully;
- understand which files belong in each directory;
- create a branch without touching generated build output;
- identify the Stage 1 task they own;
- reproduce the baseline benchmark output.

The next weekly progress file should record the completed Stage 1 work,
measured results, compiler differences, and any changes required before
beginning task-level CPU parallelism.
