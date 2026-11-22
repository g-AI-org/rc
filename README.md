# @g-ai/rc (`g-AI-org/rc`)

Shared low-latency remote control, link diagnostics, touch joystick math, video playout, and schema-driven live PID tuning library used across:

- **[`esp32-bot/balax-remote`](https://github.com/esp32-bot/balax-remote)** — Self-balancing robot PWA remote, live telemetry plotter, 3D orientation view, and live PID gain tuner
- **[`g-AI-org/gdog-remote`](https://github.com/g-AI-org/gdog-remote)** — Quadruped robot remote controller
- **[`0SFS/rc`](https://github.com/0SFS/rc)** — Flight simulator phone remote controller

## Modules

- `@g-ai/rc/diagnostics` — `LinkDiagnosticsTracker` (rolling RTT, jitter, packet drop %, TX/RX Hz)
- `@g-ai/rc/transport` — `LowLatencySession`, `resolveTargetEndpoints` (WebRTC DataChannel, WebSocket, WebSerial)
- `@g-ai/rc/controls` — `computeStickVector`, `applyDeadzoneAndExpo` (floating-origin camera joystick & dual-stick math)
- `@g-ai/rc/tuning` — `parseSettingsTable`, `TuningParameter` (live runtime parameter & PID gain synchronization)
