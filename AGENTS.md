# Agent instructions

This repository (`@g-ai/rc`) holds the shared low-latency remote control, telemetry diagnostics, touch joysticks, 6-DOF IMU sensor fusion & calibration/HIL solvers (`imuPhysics.ts`, `imuCalibration.ts`), FOSS Earth / 0SFS 3D orbit camera gestures + inertial glide (`cameraGlide.ts`, `cameraInput.ts`), used across `gdog-remote` and `esp32-bot/balax-remote`. Shared 3D UI framework components (floating log `GameLogOverlay`/`createGameLog`, docked windowing, settings, theme) live in `~/gh/g-ai/3d-ui` (`@g-ai/3d-ui`) — do not duplicate them here.

## Thin-Robot Offload & Browser-First TS/WASM Tooling

- **Offload heavy logic from embedded robots & companion remotes (like JoyX) into browser-compatible TypeScript/WASM modules here** so smartphone, tablet, and laptop web apps do the heavy lifting (calibration math, sensor fusion, HIL evaluation, GitHub release OTA fetching/compression, `IndexedDB` artifact caching, signal processing) while ESP32 firmware stays minimal, cool, and deterministic.
- Never introduce PC-only Python dependencies for tasks that can run in browser TypeScript/WASM.

## Dev servers & user testing (0SFS convention)

- Stop every temporary server and child process you start before returning; never leave a background dev server as the handoff.
- For user testing, give the user the exact `cd ~/gh/<org>/<app> && npm run dev` command to run in their own terminal:

```sh
cd ~/gh/esp32-bot/balax-remote && npm run dev
```
