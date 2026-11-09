# Soft-Body Simulation: Parallel Computing Benchmark

This project is a C++20 soft-body physics simulator and parallel-computing
benchmark based on Position-Based Dynamics (PBD) and Extended Position-Based
Dynamics (XPBD). Its purpose is to measure how the same simulation workload
behaves when expressed as:

1. scalar CPU computation;
2. single-threaded SIMD computation;
3. task-level parallel computation across CPU threads; and
4. massively parallel GPU computation.

The project is primarily about computational structure, memory access,
parallelization, synchronization, and measurement. The soft-body simulation is
the workload used to expose those issues.

A separate OpenGL renderer will eventually use the physics library for visual
demonstration. Rendering is not part of this benchmark and is intentionally
kept outside the physics and measurement concerns described here.

## Building and testing

The project uses CMake and requires a C++20 compiler. The repository includes
Debug and Release presets for the MinGW toolchain on Windows:

```text
cmake --preset mingw-debug
cmake --build --preset mingw-debug
ctest --preset mingw-debug --output-on-failure
```

The same commands work with `mingw-release` for an optimized build. The
configuration produces three targets: the `soft_body_core` static library,
the `soft_body_benchmark` executable, and the `soft_body_core_tests` test
executable. The core library is the shared boundary that Stage 1 work builds
on.

Weekly setup and implementation handoffs are recorded in
[`progress/`](progress/), beginning with
[`progress/week-1.md`](progress/week-1.md).

---

## Project scope

The simulator represents a deformable object as particles connected by
geometric constraints. A particle has a position, velocity, acceleration, and
mass or inverse mass. The object also retains its rest configuration:

- rest particle positions;
- edge connectivity;
- rest lengths;
- triangle connectivity;
- rest volume;
- constraint compliance and damping parameters.

The benchmark compares implementations that produce equivalent physical
updates. Every optimized implementation is checked against a scalar reference
with a documented floating-point tolerance.

The important distinction is between different kinds of parallelism:

| Mode | Parallel unit | CPU threads |
|---|---|---:|
| Scalar | Individual scalar operations | 1 |
| SIMD | Vector lanes processing adjacent values | 1 |
| Thread pool | Independent soft-body objects | Multiple |
| GPU compute | Device threads and workgroups | CPU-independent |

SIMD is data-level parallelism inside one thread. It is not the same as
multithreading.

---

# 1. System Architecture

## 1.1 Physics core

The physics core owns the complete simulation state and performs:

- external acceleration and time integration;
- particle position prediction;
- distance and volume constraint evaluation;
- XPBD Lagrange multiplier corrections;
- collision and boundary handling;
- velocity reconstruction;
- numerical validation and benchmark instrumentation.

The core has no responsibility for windows, cameras, OpenGL contexts, drawing,
lighting, textures, or frame presentation. It can therefore be measured as a
standalone computational workload.

The renderer that consumes this project is only an eventual client. It reads
the resulting particle or vertex positions and displays them; it does not
define the solver’s data structures, scheduling, or correctness criteria.

## 1.2 Simulation stages

One fixed simulation substep consists of:

1. applying external acceleration;
2. predicting particle velocities and positions;
3. solving constraints for several iterations;
4. applying collision or boundary constraints;
5. reconstructing velocities from the corrected positions;
6. exposing the updated state to the benchmark or client application.

The same stages are used when comparing scalar, SIMD, and CPU task-level
implementations. This keeps the physical workload constant while the
execution strategy changes.

Substeps and solver iterations are separate parameters. Substeps improve
temporal integration; iterations improve constraint convergence. Both increase
work and must remain fixed when comparing performance modes.

## 1.3 Object ownership

Each soft body owns its particle arrays, constraints, rest-state data, and
temporary solver state. A scene contains multiple bodies. This ownership
boundary is important because independent bodies can be processed concurrently
without writing the same particle memory.

Constraints that connect particles belonging to different objects, or global
collision structures shared by multiple objects, require an explicit
synchronization phase. They cannot be treated as independent object work.

---

# 2. Physics Model

The physics model supplies the dependency structure studied by the benchmark.
It is not the primary subject of the project, so the equations below are
limited to the operations required to understand the computational behavior.

## 2.1 Explicit Euler prediction

For particle i and substep duration h:

`v_pred[i] = v_old[i] + h * a[i]`

`x_pred[i] = x_old[i] + h * v_pred[i]`

With damping factor d, the velocity update used by the benchmark can be
written as:

`v_pred[i] = d * (v_old[i] + h * a[i])`

After constraint projection, the final velocity is reconstructed from the
position change:

`v_new[i] = (x_new[i] - x_old[i]) / h`

The particle update is regular and element-independent, which makes it a good
target for SoA layouts and SIMD.

## 2.2 Distance constraints

For an edge connecting particles i and j, with rest length `L0`:

`C(x) = length(x[i] - x[j]) - L0`

The normalized direction is:

`n = (x[i] - x[j]) / length(x[i] - x[j])`

The gradients are `+n` for particle i and `-n` for particle j. The solver
uses these gradients to distribute a correction according to the particles’
inverse masses.

Distance constraints resist stretching and compression. A cloth or surface
mesh normally contains many such constraints, and neighboring constraints
often share particles.

## 2.3 Volume constraints

For a consistently oriented closed triangle mesh, the signed volume is the sum
of the oriented tetrahedra formed by each triangle and the origin:

`V(x) = sum over triangles (1/6) * dot(cross(x[i], x[j]), x[k])`

The volume constraint is:

`C_V(x) = V(x) - k * V0`

where `V0` is the rest volume and `k` is a pressure or volume-scale factor.
The gradient of each triangle contribution is assembled from cross products of
the other two vertices. Volume constraints prevent a volumetric object from
collapsing while still allowing deformation.

Correct triangle orientation is required. Inconsistent winding can cause
signed-volume contributions to cancel incorrectly.

## 2.4 XPBD correction

For a scalar constraint C, gradient `g`, inverse-mass matrix `M^-1`,
compliance alpha, and time step h, XPBD uses scaled compliance:

`alpha_tilde = alpha / h^2`

The multiplier increment is:

`delta_lambda = (-C - alpha_tilde * lambda) /
                (g * M^-1 * g^T + alpha_tilde)`

The position correction is:

`delta_x = M^-1 * g^T * delta_lambda`

The corrected position is obtained by adding `delta_x` to the affected
particles. Compliance controls constraint softness and prevents stiffness from
being determined solely by iteration count.

The project notes also describe a damped equivalent using a damping parameter
and the displacement from the previous position. The implementation must use
one consistent convention for compliance, damping, and accumulated
multipliers across all constraint types.

## 2.5 Why the basic solver is sequential

The reference solver applies corrections immediately. A later constraint reads
positions already changed by earlier constraints. If two constraints share a
particle, their results depend on processing order.

This is the defining dependency:

`x_after_constraint_1 = x_before + correction_1`

Therefore constraint 2 evaluates its error using `x_after_constraint_1`, not
the original position. Running both constraints simultaneously can produce
lost updates, data races, or a different numerical method.

This behavior is commonly described as a Gauss-Seidel-style solve. It usually
converges more effectively per iteration than a fully simultaneous update, but
its immediate writes limit parallelism inside one object. Parallel execution
requires graph coloring, buffered Jacobi-style corrections, or synchronization
mechanisms such as atomics.

---

# 3. Stage 1: Data-Level Parallelism and Memory

Stage 1 improves one CPU thread by processing multiple independent values per
instruction. It includes memory-layout experiments, compiler vectorization,
OpenMP SIMD directives, and explicit AVX2 kernels.

## 3.1 Array of Structures

An Array of Structures (AoS) stores complete particles contiguously. Each
record contains position, velocity, acceleration, and mass.

AoS is convenient for scalar object-oriented code because all properties of
one particle are together. It is less efficient when a kernel processes only
one property across many particles:

- a cache line brings in fields that the kernel does not use;
- positions or velocities are separated by the record stride;
- vector loads do not naturally obtain adjacent values of one field;
- padding can increase memory traffic.

AoS is useful as a simple scalar reference and as a baseline for comparing
layout effects.

## 3.2 Structure of Arrays

A Structure of Arrays (SoA) stores each field in its own contiguous array:

`position[0..N)`, `velocity[0..N)`, and `acceleration[0..N)`.

A sequential update can stream through each array, and one vector load can
obtain adjacent values for several particles. SoA reduces unused cache-line
traffic and maps naturally to SIMD registers and GPU memory transactions.

SoA is not automatically best for every constraint operation. Constraints use
indexed access to two or more particles, so locality depends on constraint
ordering and topology. The benchmark therefore treats layout as an
experiment, not as an assumption that one representation always wins.

## 3.3 Cache locality

CPUs transfer memory in cache lines. Sequential access uses most of each line;
large strides and irregular access often use only a small portion before
loading another line.

The associated memory experiments compare row-major and column-major
traversals and several matrix-multiplication loop orders. Matrix tiling keeps
working blocks of the matrices in cache and demonstrates that mathematically
equivalent loops can have very different memory behavior.

These tests measure locality and reuse. They do not create additional CPU
threads.

## 3.4 Compiler auto-vectorization

The compiler can vectorize a loop when it can prove that iterations are
independent and that aliasing, control flow, alignment, and floating-point
semantics permit transformation.

Branchless expressions for boundary handling often allow the compiler to use
predication or vector masks instead of a branch per particle. `restrict`-style
annotations can remove assumed aliasing between input and output arrays, but
they are correctness promises. If the ranges overlap despite that promise,
the behavior is invalid.

The benchmark keeps scalar and auto-vectorized implementations separate so
that generated code and performance can be compared rather than assumed.

## 3.5 `#pragma omp simd`

The OpenMP directive used by this project is `#pragma omp simd`. It requests
SIMD vectorization of loop iterations.

It does **not**:

- create a worker team;
- distribute iterations across CPU cores;
- provide task-level parallelism;
- behave like `#pragma omp parallel for`.

An `omp simd` kernel is still one CPU thread using multiple vector lanes when
the compiler honors the directive. This distinction is essential to the
benchmark’s terminology and results.

## 3.6 Explicit AVX2

AVX2 provides 256-bit registers. A register can hold eight 32-bit floating
point values. The particle update can therefore operate on eight positions,
velocities, or accelerations at once using `__m256` values.

The explicit implementation loads eight values, performs vector additions and
multiplications, and stores the result. Boundary collisions use comparison
masks. Each lane is independently marked when a position is above the upper
limit or below the lower limit. Blend instructions select the clamped position
and reflected velocity for only the lanes that crossed a boundary.

An AVX2 kernel must account for:

- scalar cleanup when the particle count is not divisible by eight;
- correct aligned or unaligned loads;
- machines without AVX2;
- numerical differences from changed floating-point operation order;
- validation against the scalar result.

The vector width is eight floats, but the kernel is still one CPU thread.

## 3.7 Dependency-shaped benchmark kernels

The supporting benchmark includes operations with different dependency
patterns:

- SAXPY has independent element updates and is easy to vectorize.
- A dot product has independent products but a reduction dependency.
- Thresholding maps naturally to comparison masks and blends.
- Escape-time iteration has divergent per-element termination.
- Prefix scans depend on the preceding output and require a different
  parallel-prefix algorithm.
- Dependent recurrences expose instruction latency.
- Multiple independent recurrences expose instruction-level overlap.

These controls help distinguish vectorization, cache locality, branch behavior,
instruction-level parallelism, and true multithreading.

---

# 4. Stage 2: Task-Level CPU Parallelism

Stage 2 uses a custom `std::thread` pool rather than OpenMP threading. The
parallel unit is an independent soft-body object.

## 4.1 Thread pool

The pool contains persistent worker threads, a synchronized task queue, a
mutex, a condition variable, and completion handles. Workers wait for work,
remove tasks from the queue, execute them without holding the queue lock, and
wait again.

The worker count is configurable. Hardware concurrency is only a starting
point because a solver may be limited by memory bandwidth, and too many
workers can cause oversubscription.

Task failures must propagate to the caller. A worker must not silently discard
an exception or report successful completion for an incomplete simulation.

## 4.2 Object-level parallelism

Different objects can be simulated concurrently when they have disjoint:

- particle arrays;
- constraint arrays;
- temporary solver buffers;
- collision state;
- output ranges.

For example, separate soft spheres or cloth patches can occupy different
worker tasks. Since they do not write the same particle positions, this level
of parallelism does not require locks around every particle update.

The limitation is fundamental: a scene containing one large mesh has only one
independent object task. The thread pool improves throughput across objects,
not the solve time of that one mesh.

Global collision detection, shared interactions, and any cross-object
constraint require a synchronization boundary. They must not be hidden inside
otherwise independent object tasks.

## 4.3 Load balancing

Objects can differ greatly in particle and constraint count. Static assignment
can leave workers idle while one large body continues running. A shared task
queue improves balance, but queue overhead and synchronization become
significant when tasks are too small.

Meaningful measurements should report object sizes, worker count, task
duration, and total solver time. A speedup number without the workload
distribution is difficult to interpret.

## 4.4 Constraint graph coloring

To parallelize constraints within one mesh, construct a conflict graph:

- each constraint is one graph vertex;
- two graph vertices are connected when their constraints share a particle;
- constraints with the same color have no shared particle.

A proper coloring divides constraints into independent batches. One color can
be processed in parallel, followed by a synchronization barrier before the
next color. The color order preserves a Gauss-Seidel-like dependency between
batches while exposing parallel work inside each batch.

A greedy coloring algorithm assigns each constraint the lowest color not used
by its neighboring constraints. It is inexpensive and suitable for topology
preprocessing, but the number of colors directly affects performance:

- too many colors create barrier and scheduling overhead;
- too few colors are impossible when conflicts exist;
- poor ordering can produce unnecessarily many colors;
- irregular indexed memory access remains even within a color.

Coloring is not a claim that all constraints are independent. It creates
independent subsets and makes the required synchronization explicit.

## 4.5 Gauss-Seidel and Jacobi trade-off

The immediate-update solver is Gauss-Seidel-like. A colored solver keeps
immediate writes but synchronizes between independent colors.

A Jacobi-style solver reads one unchanged position state, computes corrections
into separate buffers, and applies them together. This is easier to parallelize
because constraints do not concurrently modify the live position array, but it
requires more memory and can converge differently.

Atomic position or correction updates are another option. Atomics prevent lost
writes but do not reproduce the deterministic order of the scalar solver.
Floating-point atomic operation order can vary between runs, and heavily
shared vertices can serialize the supposed parallel work.

These are different numerical algorithms, not merely different thread
implementations.

---

# 5. Stage 3: GPU Compute Architecture

GPU compute is an architectural extension of the benchmark. The current
rendering client is not the GPU physics implementation. A GPU solver would
require device-resident particle and constraint data, explicit dispatch
ordering, and a conflict-resolution strategy.

## 5.1 Device data

The GPU representation would normally contain contiguous buffers for:

- positions and previous positions;
- velocities and accelerations;
- inverse masses;
- edge indices and rest lengths;
- triangle indices and volume data;
- constraint colors and color offsets;
- temporary correction or reduction buffers.

SoA-style buffers allow neighboring threads to access neighboring values.
Constraint buffers remain indexed, so ordering constraints by color and
topological locality can improve cache behavior.

The main performance objective is to keep the state on the device. Copying all
positions to the CPU and back every frame can eliminate the benefit of GPU
parallelism.

## 5.2 Coalesced access

GPU memory transactions are most efficient when adjacent threads access
adjacent addresses. Particle integration maps naturally to this pattern:
thread i accesses particle i.

Constraint solving is less regular because one constraint accesses multiple
particle indices. Index ordering, compact buffers, color batches, and
topology-aware layout can reduce scattered access. Coalescing must be
evaluated together with occupancy, register pressure, cache usage, and
barrier cost.

## 5.3 VBO interoperability

The eventual renderer may share a position buffer with a compute API. CUDA
OpenGL interoperability or an OpenGL storage buffer can avoid a CPU staging
copy. The compute stage writes the buffer, a synchronization fence establishes
visibility, and the renderer reads it.

This still requires explicit ownership and memory synchronization. Compute and
rendering must not access the same buffer simultaneously, and interoperation
must account for device compatibility, buffer lifetime, resizing, and fallback
paths. It is an integration optimization, not a requirement of the physics
model.

## 5.4 GPU write conflicts

Multiple constraints can write the same vertex. Three standard approaches are:

### Atomic accumulation

Each constraint atomically adds its correction to a particle or correction
buffer. This prevents lost updates, but operation order is nondeterministic
and contention can be high at highly connected vertices.

### Jacobi buffering

Constraints read the current position buffer and write corrections to a
separate buffer. A later reduction accumulates corrections per particle before
the next iteration. This avoids concurrent writes to live positions but uses
additional memory and changes convergence behavior.

### Graph coloring

Only one conflict-free color is dispatched at a time. Constraints in that
color can update positions without atomics, while a device-wide barrier
separates colors. This is closer to the original Gauss-Seidel behavior but
adds preprocessing, dispatch, and barrier overhead.

The chosen strategy determines both performance and numerical behavior.

## 5.5 GPU validation

GPU results must be compared with the scalar reference using:

- maximum position and velocity error;
- distance-constraint residuals;
- volume drift;
- collision penetration;
- NaN and infinity checks;
- determinism expectations.

Bitwise equality is generally inappropriate across different execution
orders. The accepted tolerance must be defined for each workload.

---

# 6. Benchmark Methodology

## 6.1 Compared implementations

The benchmark distinguishes:

1. scalar AoS;
2. scalar SoA;
3. compiler auto-vectorized SoA;
4. OpenMP SIMD SoA;
5. explicit AVX2 SoA;
6. object-level `std::thread` execution;
7. colored or Jacobi internal-mesh execution;
8. GPU execution.

The first five isolate data representation and single-thread vectorization.
The sixth measures CPU task parallelism. The seventh and eighth address the
harder problem of parallelizing constraints that share particles.

## 6.2 Measurement protocol

Each run should use fixed:

- particle and constraint counts;
- topology;
- time step;
- substep count;
- iteration count;
- material parameters;
- initial state.

The benchmark uses a warm-up followed by repeated timed measurements. The
computation guide specifies one warm-up and ten timed repetitions as its
default pattern. Median, minimum, maximum, and throughput should be reported.

The measured region must be separated from setup and allocation. Checksums or
other observable sinks must consume results so the compiler cannot remove
apparently unused work.

## 6.3 Correctness requirements

An optimized mode is valid only when it:

- agrees with the scalar reference within its tolerance;
- produces no NaN or infinity;
- preserves valid particle and constraint indices;
- maintains acceptable distance and volume residuals;
- handles non-vector-width-aligned counts;
- reports its determinism behavior.

Floating-point reductions and reordered constraints can produce small
differences. Those differences must be measured, not hidden by omitting
correctness checks.

## 6.4 Reported metrics

Each result should include:

- particle count;
- constraint count;
- object count and size distribution;
- substeps and solver iterations;
- execution mode;
- worker count or SIMD width;
- elapsed time per step;
- throughput and speedup;
- memory layout;
- numerical error and constraint residual;
- determinism status.

This makes the benchmark useful for analyzing why a mode is faster or slower,
not merely for producing an FPS value.

---

# 7. Academic References

All referenced papers are stored in [`docs/`](docs/).

## `xpbd.pdf`

Macklin, Müller, and Chentanez, **“XPBD: Position-Based Simulation of
Compliant Constrained Dynamics.”**

Primary source for compliance, time-step-scaled compliance, accumulated
Lagrange multipliers, and the XPBD correction used in this project.

## `EG2015PBD.pdf`

Bender, Müller, and Macklin, **“Position-Based Simulation Methods in Computer
Graphics,”** Eurographics 2015 tutorial.

Reference for the general PBD framework, particles, geometric constraints,
iterative projection, distance constraints, volume constraints, and collision
constraints.

## `posBasedDyn.pdf`

Müller, Heidelberger, Hennix, and Ratcliff, **“Position Based Dynamics,”**
VRIPHYS 2006.

Foundational reference for positional constraint solving and the sequential
Gauss-Seidel-style approach used as the scalar baseline.

## `PBDBodies.pdf`

Müller, Macklin, Chentanez, Jeschke, and Kim, **“Detailed Rigid Body
Simulation with Extended Position Based Dynamics,”** 2020.

Reference for extending XPBD constraint concepts to rigid bodies, contacts,
and collision interactions. It informs the project’s constraint-oriented
architecture beyond the initial soft-body workload.

## `ecp14106004.pdf`

Fratarcangeli and Pellacini, **“Towards a Massively Parallel Solver for
Position Based Dynamics,”** SIGRAD 2014.

Primary reference for conflict graphs, greedy constraint coloring, independent
constraint clusters, colored execution, and parallel PBD scheduling.

## `jgt15_published.pdf`

Fratarcangeli and Pellacini, **“A GPU-Based Implementation of Position Based
Dynamics for Interactive Deformable Bodies,”** *Journal of Graphics Tools*.

Reference for GPU-oriented PBD data organization, constraint batching,
massively parallel deformable-body solving, and the relationship between
computation and visualization buffers.

## `uppfrta_preprint.pdf`

Macklin, Müller, Chentanez, and Kim, **“Unified Particle Physics for Real-Time
Applications,”** preprint.

Reference for using particles and constraints as a common computational
abstraction across cloth, fluids, contacts, and rigid bodies.

## Reference mapping

| Project concept | Main reference |
|---|---|
| PBD particle and constraint model | `posBasedDyn.pdf`, `EG2015PBD.pdf` |
| Distance and volume constraints | `EG2015PBD.pdf`, `posBasedDyn.pdf` |
| XPBD compliance and multipliers | `xpbd.pdf` |
| Sequential constraint dependencies | `posBasedDyn.pdf`, `xpbd.pdf` |
| Constraint conflict graphs and coloring | `ecp14106004.pdf` |
| GPU constraint batching | `ecp14106004.pdf`, `jgt15_published.pdf` |
| Unified particle-based architecture | `uppfrta_preprint.pdf`, `PBDBodies.pdf` |

---

## Summary

The project uses soft-body dynamics to study a progression of parallel
execution strategies. Regular particle updates expose cache locality and SIMD.
Independent objects expose CPU task-level parallelism. Shared particles inside
one mesh expose the synchronization problem that requires graph coloring,
buffered corrections, or atomics. GPU execution adds device memory layout,
coalescing, dispatch barriers, and optional renderer-buffer interoperability.

The benchmark’s central result is not a single implementation technique. It is
the relationship between numerical dependencies and hardware parallelism:
operations can be vectorized when their data is independent, objects can be
threaded when their state is disjoint, and internal constraints require an
algorithmic change before they can be safely executed concurrently.
