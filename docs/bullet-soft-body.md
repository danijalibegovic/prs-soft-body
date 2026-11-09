# Bullet soft-body integration

The renderer can use the separately built Bullet shared-library package through
`BulletSoftBodyAdapter`. Configure with the package location:

```powershell
cmake -S . -B build_bullet -G "MinGW Makefiles" `
  -DCMAKE_BUILD_TYPE=Debug `
  -DBULLET_PACKAGE_ROOT="C:\path\to\bullet3\build_shared\package"
```

The package must contain the four Debug import libraries under `lib/`, the
matching DLLs under `bin/`, and Bullet headers under `include/bullet/`.
Debug and Release configurations must not be mixed.

## ABI and ownership

The public adapter API is the C header
`src/include/physics/SoftBodyAdapter.h`. It exposes opaque `SoftWorld` and
`SoftBody` handles only; Bullet headers and C++ objects remain private to the
adapter implementation.

## First proof

Build and run the standalone proof:

```powershell
cmake --build build_bullet --target BulletSoftBodySmoke --parallel 4
.\build_bullet\bullet-smoke\BulletSoftBodySmoke.exe
```

The executable creates a soft cube, advances 600 fixed steps at `1/60`
second, copies node positions and indices, prints a 60-step summary, and
writes `bullet_soft_body.csv` in its working directory. The CSV schema is:

```text
step,total_ms,prediction_ms,constraints_ms,collision_ms,update_ms,node_count,face_count
```

The packaged Bullet build also prints its averaged prediction, constraints,
collision, and update timings every 60 steps. The adapter currently records
the complete step duration directly; phase timings remain zero until the
packaged Bullet instrumentation is exposed through a supported API rather than
duplicated or inferred across the DLL boundary.

The four Bullet DLLs are copied beside both `BulletSoftBodySmoke.exe` and
`TestApp.exe` by CMake post-build commands.
