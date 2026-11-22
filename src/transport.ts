import { LinkDiagnosticsTracker, LinkMetrics } from "./diagnostics";

export type TransportKind = "webrtc" | "websocket" | "webserial" | "http";

export interface ControlEnvelope<TControl = Record<string, number>> {
  seq: number;
  sentAtMs: number;
  axes: TControl;
  command?: string;
}

export interface TelemetryEnvelope<TState = Record<string, number | string>> {
  seq: number;
  echoSentAtMs?: number;
  state: TState;
}

/**
 * Normalize a user-supplied host/IP or URL into HTTP and WebSocket endpoints.
 * Shared across gdog-remote and balax-remote.
 */
export function resolveTargetEndpoints(rawInput: string, defaultPort = 80): {
  host: string;
  httpBase: string;
  wsUrl: string;
} {
  const trimmed = rawInput.trim();
  if (!trimmed) {
    return { host: "192.168.4.1", httpBase: "http://192.168.4.1", wsUrl: "ws://192.168.4.1/ws" };
  }
  const hasScheme = /^https?:\/\//i.test(trimmed) || /^wss?:\/\//i.test(trimmed);
  const urlStr = hasScheme ? trimmed : `http://${trimmed}`;
  try {
    const u = new URL(urlStr);
    if (!u.port && !hasScheme && defaultPort !== 80) {
      u.port = String(defaultPort);
    }
    const isSecure = u.protocol === "https:" || u.protocol === "wss:";
    const httpProto = isSecure ? "https:" : "http:";
    const wsProto = isSecure ? "wss:" : "ws:";
    const hostPort = u.host;
    return {
      host: hostPort,
      httpBase: `${httpProto}//${hostPort}`,
      wsUrl: `${wsProto}//${hostPort}/ws`,
    };
  } catch {
    return {
      host: trimmed,
      httpBase: `http://${trimmed}`,
      wsUrl: `ws://${trimmed}/ws`,
    };
  }
}

export class LowLatencySession<
  TControl = Record<string, number>,
  TState = Record<string, number | string>
> {
  public readonly diagnostics = new LinkDiagnosticsTracker();
  private seq = 0;
  private ws: WebSocket | null = null;
  private listeners = new Set<(telemetry: TelemetryEnvelope<TState>, metrics: LinkMetrics) => void>();

  connectWebSocket(wsUrl: string): void {
    this.disconnect();
    const ws = new WebSocket(wsUrl);
    this.ws = ws;
    ws.onopen = () => {
      this.diagnostics.connected = true;
    };
    ws.onclose = () => {
      this.diagnostics.connected = false;
    };
    ws.onmessage = (ev) => {
      try {
        const msg = JSON.parse(typeof ev.data === "string" ? ev.data : "") as TelemetryEnvelope<TState>;
        const now = performance.now();
        const rtt = typeof msg.echoSentAtMs === "number" ? now - msg.echoSentAtMs : undefined;
        this.diagnostics.recordRx(msg.seq, rtt, now);
        const snap = this.diagnostics.snapshot(now);
        for (const cb of this.listeners) cb(msg, snap);
      } catch {
        // Ignore malformed packets
      }
    };
  }

  sendControl(axes: TControl, command?: string): ControlEnvelope<TControl> {
    const now = performance.now();
    const packet: ControlEnvelope<TControl> = {
      seq: ++this.seq,
      sentAtMs: now,
      axes,
      command,
    };
    this.diagnostics.recordTx(now);
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(packet));
    }
    return packet;
  }

  onTelemetry(cb: (telemetry: TelemetryEnvelope<TState>, metrics: LinkMetrics) => void): () => void {
    this.listeners.add(cb);
    return () => this.listeners.delete(cb);
  }

  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
    this.diagnostics.connected = false;
  }
}
