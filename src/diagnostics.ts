/**
 * Shared low-latency link diagnostics (RTT, jitter, packet drop rate, TX/RX Hz)
 * used across 0SFS/rc, g-AI-org/gdog-remote, and esp32-bot/balax-remote.
 */
export interface LinkMetrics {
  rttMs: number | null;
  jitterMs: number | null;
  txHz: number;
  rxHz: number;
  lossPercent: number;
  connected: boolean;
}

export class LinkDiagnosticsTracker {
  private rttSamples: number[] = [];
  private txTimestamps: number[] = [];
  private rxTimestamps: number[] = [];
  private lastRxSeq: number | null = null;
  private receivedCount = 0;
  private lostCount = 0;
  public connected = false;

  recordTx(nowMs = performance.now()): void {
    this.txTimestamps.push(nowMs);
    this.prune(this.txTimestamps, nowMs);
  }

  recordRx(seq?: number, rttMs?: number, nowMs = performance.now()): void {
    this.rxTimestamps.push(nowMs);
    this.prune(this.rxTimestamps, nowMs);
    this.receivedCount++;
    if (typeof seq === "number" && this.lastRxSeq !== null && seq > this.lastRxSeq + 1) {
      this.lostCount += seq - this.lastRxSeq - 1;
    }
    if (typeof seq === "number") {
      this.lastRxSeq = seq;
    }
    if (typeof rttMs === "number" && Number.isFinite(rttMs) && rttMs >= 0) {
      this.rttSamples.push(rttMs);
      if (this.rttSamples.length > 60) this.rttSamples.shift();
    }
  }

  snapshot(nowMs = performance.now()): LinkMetrics {
    this.prune(this.txTimestamps, nowMs);
    this.prune(this.rxTimestamps, nowMs);
    const rttMs =
      this.rttSamples.length > 0
        ? this.rttSamples.reduce((a, b) => a + b, 0) / this.rttSamples.length
        : null;
    let jitterMs: number | null = null;
    if (this.rttSamples.length > 1 && rttMs !== null) {
      const variance =
        this.rttSamples.reduce((acc, s) => acc + (s - rttMs) ** 2, 0) /
        this.rttSamples.length;
      jitterMs = Math.sqrt(variance);
    }
    const total = this.receivedCount + this.lostCount;
    return {
      rttMs,
      jitterMs,
      txHz: this.txTimestamps.length,
      rxHz: this.rxTimestamps.length,
      lossPercent: total > 0 ? (this.lostCount / total) * 100 : 0,
      connected: this.connected,
    };
  }

  private prune(arr: number[], nowMs: number): void {
    while (arr.length > 0 && nowMs - arr[0] > 1000) {
      arr.shift();
    }
  }
}
