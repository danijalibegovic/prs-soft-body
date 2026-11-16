# Demo Scenes

The executable loads the XPBD soft-body showcase at startup.

## XPBD Soft Bodies

**Source:** `test-app/src/XpbdSoftBodyScene.cpp`

The scene combines a deformable cube with a hollow elastic sphere. The ball is
a welded geodesic icosphere (frequency 7: 492 vertices and 980 triangles), with
area-weighted particle masses, edge and bending distance constraints, and one
closed-surface XPBD volume constraint. Its initial center is offset past the
cube's support edge so it rolls off rather than settling in a symmetric,
centered contact. It runs with a fixed `1/120` second timestep.

**What to check:**

- The sphere falls, deforms on impact, bounces slightly, and retains most of
  its volume.
- The sphere's shading remains smooth as it deforms.
- Both bodies remain supported by the ground.

The standalone Bullet adapter proof and telemetry format are documented in
[bullet-soft-body.md](./bullet-soft-body.md).

## Controls

| Key / Input | Action |
|---|---|
| `W` / `A` / `S` / `D` | Move |
| `SPACE` / `LEFT CTRL` | Move up / down |
| `LEFT SHIFT` | Sprint |
| Mouse (while captured) | Look around |
| `TAB` | Toggle mouse capture |
| `F1` / `F2` / `F3` | Camera mode |
| `X` | Toggle inspector sidebar |
| `Z` | Toggle wireframe |
| `F11` | Toggle fullscreen |
