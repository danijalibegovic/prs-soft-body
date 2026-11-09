# Demo Scene

The executable intentionally loads one scene: a Bullet soft cube showcase.

## Bullet Soft Cube

**Source:** `test-app/src/BulletSoftBodyScene.cpp`

The scene runs Bullet through the C ABI adapter with a fixed `1/60` second
timestep. A convex-hull soft cube falls under gravity onto a static ground
plane, deforms during impact, and settles instead of passing through the
ground.

**What to check:**

- The cube falls from above the ground.
- The cube visibly jiggles/deforms on contact.
- The cube remains supported by the ground.
- Bullet prints a timing summary every 60 physics steps.

The standalone adapter proof and telemetry format are documented in
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
