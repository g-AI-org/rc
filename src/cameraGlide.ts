/**
 * FOSS Earth & 0SFS camera glide (`engine/src/input/inertialCameraController.ts`
 * and `0SFS/controls/src/cameraGlide.ts`):
 *
 * - Every zoom and glide-fed orbit input adds to `owedA` / `owedB` and eases in
 *   over the next frames at `(1 - decay)` per 60 Hz frame, so mouse wheel notches,
 *   trackpad pinches, and Option/Alt-swipe zooms are smoothly interpolated.
 * - Gestures that do not continue after the fingers lift (mouse drag, trackpad pinch,
 *   touch orbit/pinch) arm their release momentum and feed an exponential release tail
 *   (`RELEASE_TAIL_MS = 1100 ms`, `exp(-3 * t / 1100)`) after release.
 */

export type GlideChannel = "orbit" | "zoom";

const FRAME_MS = 1000 / 60;
export const DEFAULT_GLIDE_DECAY = 0.82;
const RELEASE_WINDOW_MS = 120;
const MAX_FRAME_MS = 100;
export const PINCH_RELEASE_GAP_MS = 48;
const RELEASE_TAIL_MS = 1100;

const MAX_STEP: Record<GlideChannel, number> = {
  orbit: (12 * Math.PI) / 180,
  zoom: 0.14,
};

const REST_STEP: Record<GlideChannel, number> = {
  orbit: (0.002 * Math.PI) / 180,
  zoom: 0.00005,
};

const CHANNELS: readonly GlideChannel[] = ["orbit", "zoom"];

export interface CameraGlideOptions {
  /** `camera.inertiaDecay`, read on every input and step (0 = instant, 0.82 = default). */
  decay(): number;
  /** Turns the camera by `yaw` and `pitch`, radians. */
  orbit(yaw: number, pitch: number): void;
  /** Multiplies the camera distance by `factor` (e.g. `radius *= factor`). */
  zoom(factor: number): void;
  /** Called when new glide motion arrives so the host can schedule a frame if needed. */
  wake?(): void;
  /** Monotonic milliseconds (`performance.now` by default). */
  now?(): number;
}

export interface CameraGlide {
  /** Queue orbit motion (`yaw`, `pitch` in radians) through the inertial glide. */
  orbit(yaw: number, pitch: number): void;
  /** Queue zoom factor (`factor > 0`) through the inertial glide. */
  zoom(factor: number): void;
  /** Queue logarithmic zoom delta (`logFactor = Math.log(factor)`) through the inertial glide. */
  zoomLog(logFactor: number): void;
  /**
   * Arm release momentum on `channel` for a gesture that stops when fingers lift.
   * With `gapMs`, automatically releases once input has been quiet for `gapMs` (e.g. trackpad pinch).
   */
  arm(channel: GlideChannel, gapMs?: number): void;
  /** Release the armed gesture on `channel` so its tail glides on at release speed. */
  release(channel: GlideChannel): void;
  /** Stop all channels immediately (e.g. when a finger lands). */
  cancel(): void;
  /** Stop only channels that are not currently armed (e.g. when a second finger joins). */
  hold(): void;
  /** Advance the glide to `now` ms. Returns true while any channel still has motion or an armed release. */
  step(now?: number): boolean;
  /** True while `channel` (or any channel) is still moving or waiting to release. */
  isActive(channel?: GlideChannel): boolean;
}

interface Sample {
  t: number;
  a: number;
  b: number;
}

interface ChannelState {
  owedA: number;
  owedB: number;
  armed: boolean;
  gapMs: number | null;
  recent: Sample[];
  last: Sample | null;
  tail: { a: number; b: number; startedAt: number; fedMs: number } | null;
}

const idle = (): ChannelState => ({
  owedA: 0,
  owedB: 0,
  armed: false,
  gapMs: null,
  recent: [],
  last: null,
  tail: null,
});

const tailShare = (ms: number): number =>
  (RELEASE_TAIL_MS / 3) * (1 - Math.exp((-3 * ms) / RELEASE_TAIL_MS));

export function createCameraGlide(options: CameraGlideOptions): CameraGlide {
  const clock = options.now ?? (() => performance.now());
  const channels: Record<GlideChannel, ChannelState> = {
    orbit: idle(),
    zoom: idle(),
  };
  let steppedAt: number | null = null;
  let fresh = false;

  const decay = (): number => {
    const value = options.decay();
    return Number.isFinite(value)
      ? Math.min(0.999, Math.max(0, value))
      : DEFAULT_GLIDE_DECAY;
  };

  const apply = (channel: GlideChannel, a: number, b: number): void => {
    if (channel === "orbit") {
      if (a !== 0 || b !== 0) options.orbit(a, b);
    } else if (a !== 0) {
      options.zoom(Math.exp(a));
    }
  };

  const busy = (state: ChannelState): boolean =>
    state.owedA !== 0 ||
    state.owedB !== 0 ||
    state.tail !== null ||
    (state.armed && state.gapMs !== null);

  const isActive = (channel?: GlideChannel): boolean =>
    channel ? busy(channels[channel]) : busy(channels.orbit) || busy(channels.zoom);

  const bound = (channel: GlideChannel, value: number, keep: number): number => {
    const most = MAX_STEP[channel] / (1 - keep);
    return Math.max(-most, Math.min(most, value));
  };

  const remember = (samples: Sample[], sample: Sample): Sample[] => {
    const kept = samples.filter((item) => item.t >= sample.t - RELEASE_WINDOW_MS);
    kept.push(sample);
    return kept;
  };

  const beginTail = (state: ChannelState, endedAt: number, startedAt: number): void => {
    const kept = state.recent.filter((sample) => sample.t >= endedAt - RELEASE_WINDOW_MS);
    state.armed = false;
    state.gapMs = null;
    state.recent = [];
    if (kept.length === 0) return;
    let a = 0;
    let b = 0;
    for (const sample of kept) {
      a += sample.a;
      b += sample.b;
    }
    if (a === 0 && b === 0) return;
    const span = Math.max(FRAME_MS, endedAt - kept[0]!.t);
    state.tail = { a: a / span, b: b / span, startedAt, fedMs: 0 };
  };

  const push = (channel: GlideChannel, a: number, b: number): void => {
    if (!Number.isFinite(a) || !Number.isFinite(b) || (a === 0 && b === 0)) return;
    const keep = decay();
    if (keep === 0) {
      apply(channel, a, b);
      return;
    }
    const at = clock();
    if (!isActive() && (steppedAt === null || at - steppedAt > FRAME_MS * 2)) {
      steppedAt = at;
      fresh = true;
    }
    const state = channels[channel];
    state.tail = null;
    const sample = { t: at, a, b };
    state.last = sample;
    if (state.armed) state.recent = remember(state.recent, sample);
    state.owedA = bound(channel, state.owedA + a, keep);
    state.owedB = bound(channel, state.owedB + b, keep);
    options.wake?.();
  };

  const feedTail = (
    channel: GlideChannel,
    state: ChannelState,
    now: number,
    keep: number
  ): void => {
    const tail = state.tail!;
    const elapsed = Math.min(RELEASE_TAIL_MS, Math.max(0, now - tail.startedAt));
    if (elapsed > tail.fedMs) {
      const share = tailShare(elapsed) - tailShare(tail.fedMs);
      state.owedA = bound(channel, state.owedA + tail.a * share, keep);
      state.owedB = bound(channel, state.owedB + tail.b * share, keep);
      tail.fedMs = elapsed;
    }
    if (elapsed >= RELEASE_TAIL_MS) state.tail = null;
  };

  return {
    orbit: (yaw, pitch) => push("orbit", yaw, pitch),
    zoom: (factor) => {
      if (Number.isFinite(factor) && factor > 0) push("zoom", Math.log(factor), 0);
    },
    zoomLog: (logFactor) => {
      if (Number.isFinite(logFactor) && logFactor !== 0) push("zoom", logFactor, 0);
    },
    arm(channel, gapMs) {
      if (decay() === 0) return;
      const state = channels[channel];
      const at = clock();
      if (!state.armed && state.last && at - state.last.t <= FRAME_MS * 2) {
        state.recent = remember(state.recent, state.last);
      }
      state.armed = true;
      state.gapMs = gapMs !== undefined && gapMs >= 0 ? gapMs : null;
      if (state.gapMs !== null) options.wake?.();
    },
    release(channel) {
      const state = channels[channel];
      if (!state.armed) return;
      const at = clock();
      const from = Math.max(at - MAX_FRAME_MS, steppedAt ?? at, state.last?.t ?? at);
      beginTail(state, at, Math.min(at, from));
      if (state.tail) options.wake?.();
    },
    cancel() {
      for (const channel of CHANNELS) channels[channel] = idle();
      fresh = false;
    },
    hold() {
      for (const channel of CHANNELS) {
        if (!channels[channel].armed) channels[channel] = idle();
      }
    },
    step(now = clock()) {
      const previous = steppedAt ?? now - FRAME_MS;
      let elapsed = steppedAt === null ? FRAME_MS : Math.max(0, now - steppedAt);
      if (fresh) {
        elapsed = Math.max(elapsed, FRAME_MS);
        fresh = false;
      }
      steppedAt = now;
      const keep = decay();
      const kept = Math.pow(keep, elapsed / FRAME_MS);
      for (const channel of CHANNELS) {
        const state = channels[channel];
        if (
          state.armed &&
          state.gapMs !== null &&
          state.last &&
          now - state.last.t >= state.gapMs
        ) {
          beginTail(
            state,
            state.last.t,
            Math.min(now, Math.max(now - MAX_FRAME_MS, previous, state.last.t))
          );
        }
        if (state.tail) feedTail(channel, state, now, keep);
        if (state.owedA === 0 && state.owedB === 0) continue;
        let a = state.owedA * (1 - kept);
        let b = state.owedB * (1 - kept);
        state.owedA -= a;
        state.owedB -= b;
        if (
          !state.tail &&
          Math.max(Math.abs(state.owedA), Math.abs(state.owedB)) * (1 - keep) <
            REST_STEP[channel]
        ) {
          const most = REST_STEP[channel] / (1 - DEFAULT_GLIDE_DECAY);
          a += Math.max(-most, Math.min(most, state.owedA));
          b += Math.max(-most, Math.min(most, state.owedB));
          state.owedA = 0;
          state.owedB = 0;
        }
        apply(channel, a, b);
      }
      return isActive();
    },
    isActive,
  };
}

export interface TouchSurfaceFrame {
  fingers: number | null;
  yaw: number;
  pitch: number;
  zoom: number;
}

export function createTouchSurfaceGlide(
  glide: Pick<CameraGlide, "orbit" | "zoom" | "arm" | "release" | "cancel" | "hold">
): { frame(frame: TouchSurfaceFrame): void } {
  let before: number | null = null;
  return {
    frame({ fingers, yaw, pitch, zoom }) {
      if (fingers !== null && before !== null && fingers > before) {
        if (before === 0) glide.cancel();
        else glide.hold();
      }
      const held = fingers === null ? null : Math.max(fingers, before ?? 0) > 0;
      const gap = held === null ? PINCH_RELEASE_GAP_MS : undefined;
      if (yaw !== 0 || pitch !== 0) {
        glide.orbit(yaw, pitch);
        if (held !== false) glide.arm("orbit", gap);
      }
      if (zoom !== 1 && Number.isFinite(zoom) && zoom > 0) {
        glide.zoom(zoom);
        if (held !== false) glide.arm("zoom", gap);
      }
      const left = fingers ?? 0;
      if (before !== null && left < before) {
        if (left < 2) glide.release("zoom");
        if (left === 0) glide.release("orbit");
      }
      before = fingers;
    },
  };
}
