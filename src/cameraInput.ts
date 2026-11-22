import {
  createCameraGlide,
  createTouchSurfaceGlide,
  DEFAULT_GLIDE_DECAY,
  PINCH_RELEASE_GAP_MS,
  type TouchSurfaceFrame,
} from "./cameraGlide";

export type CameraInputMode = "mouse" | "trackpad";

export interface CameraInputSettings {
  mode: CameraInputMode;
  dragSensitivity: number;
  zoomSensitivity: number;
  invertYaw: boolean;
  invertPitch: boolean;
  glideDecay: number;
  mouseDragThresholdPx?: number;
}

export const DEFAULT_CAMERA_INPUT_SETTINGS: CameraInputSettings = {
  mode: "trackpad",
  dragSensitivity: 1,
  zoomSensitivity: 1,
  invertYaw: false,
  invertPitch: false,
  glideDecay: DEFAULT_GLIDE_DECAY,
  mouseDragThresholdPx: 5,
};

const CAMERA_STORAGE_KEY = "g-ai.camera-input-settings";

export function readSavedCameraInputSettings(): CameraInputSettings {
  if (typeof window === "undefined") return { ...DEFAULT_CAMERA_INPUT_SETTINGS };
  try {
    const raw = window.localStorage.getItem(CAMERA_STORAGE_KEY);
    if (!raw) return { ...DEFAULT_CAMERA_INPUT_SETTINGS };
    const parsed = JSON.parse(raw) as Partial<CameraInputSettings>;
    return {
      mode: parsed.mode === "mouse" ? "mouse" : "trackpad",
      dragSensitivity:
        typeof parsed.dragSensitivity === "number" && Number.isFinite(parsed.dragSensitivity)
          ? Math.max(0.2, Math.min(3, parsed.dragSensitivity))
          : 1,
      zoomSensitivity:
        typeof parsed.zoomSensitivity === "number" && Number.isFinite(parsed.zoomSensitivity)
          ? Math.max(0.2, Math.min(3, parsed.zoomSensitivity))
          : 1,
      invertYaw: Boolean(parsed.invertYaw),
      invertPitch: Boolean(parsed.invertPitch),
      glideDecay:
        typeof parsed.glideDecay === "number" && Number.isFinite(parsed.glideDecay)
          ? Math.max(0, Math.min(0.96, parsed.glideDecay))
          : DEFAULT_GLIDE_DECAY,
      mouseDragThresholdPx:
        typeof parsed.mouseDragThresholdPx === "number" &&
        Number.isFinite(parsed.mouseDragThresholdPx)
          ? Math.max(1, Math.min(24, parsed.mouseDragThresholdPx))
          : 5,
    };
  } catch {
    return { ...DEFAULT_CAMERA_INPUT_SETTINGS };
  }
}

export function writeSavedCameraInputSettings(settings: CameraInputSettings): void {
  if (typeof window === "undefined") return;
  try {
    window.localStorage.setItem(CAMERA_STORAGE_KEY, JSON.stringify(settings));
  } catch {
    // ignore storage errors
  }
}

export interface OrbitCameraAdapter {
  /** Apply orbit yaw/pitch deltas in radians. */
  orbitBy(yawDeltaRad: number, pitchDeltaRad: number): void;
  /** Apply zoom log delta (`radius *= Math.exp(zoomLogDelta)`). */
  zoomBy(zoomLogDelta: number): void;
}

/**
 * Duck-typed Babylon.js `ArcRotateCamera` interface so any Babylon project can pass
 * its `ArcRotateCamera` directly (or via `createBabylonArcRotateAdapter(camera)`)
 * without duplicating alpha/beta/radius math or direction signs.
 */
export interface BabylonArcRotateCameraLike {
  alpha: number;
  beta: number;
  radius: number;
  lowerBetaLimit?: number | null;
  upperBetaLimit?: number | null;
  lowerRadiusLimit?: number | null;
  upperRadiusLimit?: number | null;
}

/**
 * Creates a standardized `OrbitCameraAdapter` for a Babylon.js `ArcRotateCamera`
 * so all Babylon.js projects share identical yaw (`alpha`), pitch (`beta`), and
 * zoom (`radius`) clamping and sign conventions.
 */
export function createBabylonArcRotateAdapter(
  camera: BabylonArcRotateCameraLike
): OrbitCameraAdapter {
  return {
    orbitBy(yawDeltaRad, pitchDeltaRad) {
      camera.alpha += yawDeltaRad;
      const minBeta = camera.lowerBetaLimit ?? 0.08;
      const maxBeta = camera.upperBetaLimit ?? Math.PI - 0.08;
      camera.beta = Math.max(minBeta, Math.min(maxBeta, camera.beta + pitchDeltaRad));
    },
    zoomBy(zoomLogDelta) {
      const minR = camera.lowerRadiusLimit ?? 0.05;
      const maxR = camera.upperRadiusLimit ?? 1000;
      camera.radius = Math.max(minR, Math.min(maxR, camera.radius * Math.exp(zoomLogDelta)));
    },
  };
}

function isBabylonArcRotateCamera(
  target: OrbitCameraAdapter | BabylonArcRotateCameraLike
): target is BabylonArcRotateCameraLike {
  return (
    typeof (target as BabylonArcRotateCameraLike).alpha === "number" &&
    typeof (target as BabylonArcRotateCameraLike).beta === "number" &&
    typeof (target as BabylonArcRotateCameraLike).radius === "number"
  );
}

/**
 * Duck-typed Babylon.js scene interface for checking whether a pointerdown
 * landed on an interactive 3D mesh (one with an `actionManager` or
 * custom `metadata.clickable`).
 */
export interface BabylonPickableSceneLike {
  pick?(
    x: number,
    y: number,
    predicate?: (mesh: {
      isPickable?: boolean;
      isVisible?: boolean;
      isEnabled?: () => boolean;
      actionManager?: object | null;
      metadata?: { clickable?: boolean } | null;
    }) => boolean
  ): { hit?: boolean; pickedMesh?: unknown } | null;
}

export interface AttachOrbitCameraInputOptions {
  getSettings?: () => CameraInputSettings;
  /**
   * Optional Babylon.js `Scene` (or custom predicate). When supplied, a pointer press
   * on an interactive/clickable Babylon mesh (one with an `actionManager` or
   * `metadata.clickable === true`) is yielded to Babylon unless dragged, and
   * clicks below `mouseDragThresholdPx` always pass through to Babylon picking.
   */
  scene?: BabylonPickableSceneLike;
  /**
   * Custom predicate returning true if the scene point at `(clientX, clientY)`
   * has a clickable object that should capture the pointer press instead of orbiting.
   */
  isClickableAt?: (clientX: number, clientY: number, button: number) => boolean;
  onUserOrbit?: () => void;
  onOrbitGestureEnd?: () => void;
}

export function isBabylonClickableAt(
  scene: BabylonPickableSceneLike | undefined,
  canvas: HTMLCanvasElement,
  clientX: number,
  clientY: number
): boolean {
  if (!scene || typeof scene.pick !== "function") return false;
  const rect = canvas.getBoundingClientRect();
  const x = clientX - rect.left;
  const y = clientY - rect.top;
  const hit = scene.pick(x, y, (mesh) => {
    if (!mesh || mesh.isPickable === false || mesh.isVisible === false) return false;
    if (typeof mesh.isEnabled === "function" && !mesh.isEnabled()) return false;
    return Boolean(mesh.actionManager || mesh.metadata?.clickable);
  });
  return Boolean(hit?.hit && hit.pickedMesh);
}

export function attachOrbitCameraInput(
  canvas: HTMLCanvasElement,
  cameraTarget: OrbitCameraAdapter | BabylonArcRotateCameraLike,
  options: AttachOrbitCameraInputOptions = {}
): () => void {
  const camera: OrbitCameraAdapter = isBabylonArcRotateCamera(cameraTarget)
    ? createBabylonArcRotateAdapter(cameraTarget)
    : cameraTarget;
  const owner = canvas.ownerDocument.defaultView ?? window;
  const getSettings = options.getSettings ?? readSavedCameraInputSettings;
  const safari =
    typeof owner !== "undefined" &&
    "GestureEvent" in owner &&
    /Safari/i.test(owner.navigator?.userAgent ?? "") &&
    !/Chrom(e|ium)|Android/i.test(owner.navigator?.userAgent ?? "");

  let rafId: number | null = null;

  const glide = createCameraGlide({
    decay: () => getSettings().glideDecay,
    orbit: (yaw, pitch) => {
      camera.orbitBy(yaw, pitch);
    },
    zoom: (factor) => {
      camera.zoomBy(Math.log(factor));
    },
    wake: () => {
      ensureGlideLoop();
    },
  });

  function ensureGlideLoop(): void {
    if (rafId !== null) return;
    const tick = () => {
      rafId = null;
      if (glide.step(performance.now())) {
        rafId = owner.requestAnimationFrame(tick);
      }
    };
    rafId = owner.requestAnimationFrame(tick);
  }

  const makeTouchGlide = () => {
    const surface = createTouchSurfaceGlide({
      orbit: (yaw, pitch) => glide.orbit(yaw, pitch),
      zoom: (factor) => glide.zoom(factor),
      arm: (channel, gap) => glide.arm(channel, gap),
      release: (channel) => glide.release(channel),
      cancel: () => glide.cancel(),
      hold: () => glide.hold(),
    });
    surface.frame({ fingers: 0, yaw: 0, pitch: 0, zoom: 1 });
    return surface;
  };
  let touchGlide = makeTouchGlide();

  // Mouse pointer drag state (FOSS Earth mouseController.ts pattern):
  // Works in both "trackpad" and "mouse" mode for left (0) and right (2) buttons.
  // On pointerdown we do NOT consume the event so Babylon picking sees clicks.
  // Once movement exceeds mouseDragThresholdPx (5px), we commit to an orbit drag
  // and feed the inertial glide so releasing the mouse glides smoothly to a stop.
  let activePress: {
    pointerId: number;
    button: 0 | 2;
    startX: number;
    startY: number;
    prevX: number;
    prevY: number;
    isDragging: boolean;
  } | null = null;

  let gestureScale: number | null = null;
  let controlHeld = false;
  const ownedTouches = new Set<number>();
  let touch: { ids: number[]; x: number; y: number; distance: number } | null = null;
  let touchWheelCooldownUntil = 0;
  let swipeEndTimer: ReturnType<typeof setTimeout> | null = null;

  const orbitDelta = (dx: number, dy: number): { yaw: number; pitch: number } => {
    const s = getSettings();
    const scale = 0.005 * s.dragSensitivity;
    const yawSign = s.invertYaw ? -1 : 1;
    const pitchSign = s.invertPitch ? -1 : 1;
    return {
      yaw: dx * scale * yawSign,
      pitch: dy * scale * pitchSign,
    };
  };

  const zoomFactor = (rawFactor: number): number => {
    const s = getSettings();
    return Math.pow(rawFactor, s.zoomSensitivity);
  };

  const queueZoom = (rawFactor: number, armPinchRelease: boolean): void => {
    const factor = zoomFactor(rawFactor);
    if (!Number.isFinite(factor) || factor <= 0 || factor === 1) return;
    glide.zoom(factor);
    if (armPinchRelease) {
      glide.arm("zoom", PINCH_RELEASE_GAP_MS);
    }
    ensureGlideLoop();
  };

  const endSwipeWhenQuiet = (): void => {
    if (swipeEndTimer !== null) clearTimeout(swipeEndTimer);
    swipeEndTimer = setTimeout(() => {
      swipeEndTimer = null;
      if (!activePress && touch === null) {
        options.onOrbitGestureEnd?.();
      }
    }, PINCH_RELEASE_GAP_MS);
  };

  const touchBlocksWheel = (): boolean =>
    ownedTouches.size > 0 || performance.now() < touchWheelCooldownUntil;

  // ── Pointer (Left & Right Mouse Button Drag + Babylon Click Pass-Through) ──
  const onPointerDown = (e: PointerEvent): void => {
    if (e.pointerType !== "mouse") return;
    if (e.button !== 0 && e.button !== 2) return;

    // Check if the press landed on a clickable Babylon.js object
    if (
      options.isClickableAt?.(e.clientX, e.clientY, e.button) ||
      isBabylonClickableAt(options.scene, canvas, e.clientX, e.clientY)
    ) {
      return;
    }

    // Do NOT stop propagation on pointerdown so Babylon picking / click handlers still see clicks
    activePress = {
      pointerId: e.pointerId,
      button: e.button as 0 | 2,
      startX: e.clientX,
      startY: e.clientY,
      prevX: e.clientX,
      prevY: e.clientY,
      isDragging: false,
    };
  };

  const onPointerMove = (e: PointerEvent): void => {
    if (!activePress || e.pointerId !== activePress.pointerId) return;
    const mask = activePress.button === 0 ? 1 : 2;
    if ((e.buttons & mask) === 0) {
      finishPointerPress(e);
      return;
    }

    const dx = e.clientX - activePress.prevX;
    const dy = e.clientY - activePress.prevY;
    const distFromStart = Math.hypot(
      e.clientX - activePress.startX,
      e.clientY - activePress.startY
    );
    const threshold = getSettings().mouseDragThresholdPx ?? 5;

    if (!activePress.isDragging) {
      if (distFromStart < threshold) return;
      activePress.isDragging = true;
      glide.cancel();
      try {
        canvas.setPointerCapture(e.pointerId);
      } catch {
        // ignore if pointer capture unavailable
      }
    }

    e.preventDefault();
    e.stopImmediatePropagation();
    activePress.prevX = e.clientX;
    activePress.prevY = e.clientY;

    // Left/right click-drag moves in the grab/pull direction (opposite sign to raw wheel deltaX/deltaY)
    const { yaw, pitch } = orbitDelta(-dx, -dy);
    if (yaw !== 0 || pitch !== 0) {
      glide.orbit(yaw, pitch);
      glide.arm("orbit");
      ensureGlideLoop();
      options.onUserOrbit?.();
    }
  };

  const finishPointerPress = (e?: PointerEvent): void => {
    if (!activePress) return;
    const wasDragging = activePress.isDragging;
    const pid = activePress.pointerId;
    activePress = null;
    if (wasDragging) {
      e?.preventDefault();
      e?.stopImmediatePropagation();
      glide.release("orbit");
      ensureGlideLoop();
      options.onOrbitGestureEnd?.();
    }
    if (canvas.hasPointerCapture?.(pid)) {
      try {
        canvas.releasePointerCapture(pid);
      } catch {
        // ignore
      }
    }
  };

  const onPointerUp = (e: PointerEvent): void => {
    if (!activePress || e.pointerId !== activePress.pointerId) return;
    finishPointerPress(e);
  };

  const onContextMenu = (e: MouseEvent): void => {
    e.preventDefault();
  };

  // ── Wheel & Trackpad Gestures (FOSS Earth / 0SFS) ──
  const onWheel = (e: WheelEvent): void => {
    e.preventDefault();
    if (touchBlocksWheel()) return;

    const unit =
      e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? canvas.clientHeight || 800 : 1;
    const s = getSettings();

    if (s.mode === "mouse") {
      // Mouse wheel notch zoom: eased in smoothly over frames by cameraGlide
      queueZoom(Math.exp(Math.max(-1, Math.min(1, e.deltaY * unit * 0.0025))), true);
    } else if (e.altKey) {
      // Option/Alt + two-finger swipe zooms with release momentum
      queueZoom(Math.exp(Math.max(-1, Math.min(1, e.deltaY * unit * 0.01))), true);
    } else if (e.ctrlKey && !controlHeld) {
      // Trackpad pinch (Chrome/Firefox ctrl+wheel): zooms with release momentum
      if (!safari) {
        queueZoom(Math.exp(Math.max(-1, Math.min(1, e.deltaY * unit * 0.01))), true);
      }
    } else if (gestureScale === null) {
      // Two-finger trackpad swipe orbits directly (macOS trackpad supplies swipe momentum)
      const { yaw, pitch } = orbitDelta(e.deltaX * unit, e.deltaY * unit);
      camera.orbitBy(yaw, pitch);
      options.onUserOrbit?.();
      endSwipeWhenQuiet();
    }
  };

  // ── Safari Native Trackpad Pinch GestureEvents ──
  const onGestureStart = (ev: Event): void => {
    const e = ev as Event & { scale?: number };
    e.preventDefault();
    gestureScale =
      getSettings().mode === "trackpad" && ownedTouches.size === 0
        ? e.scale ?? 1
        : null;
  };

  const onGestureChange = (ev: Event): void => {
    const e = ev as Event & { scale?: number };
    e.preventDefault();
    const scale = e.scale;
    if (
      getSettings().mode === "trackpad" &&
      gestureScale !== null &&
      scale &&
      Number.isFinite(scale) &&
      scale > 0
    ) {
      queueZoom(gestureScale / scale, true);
      gestureScale = scale;
    }
  };

  const onGestureEnd = (ev: Event): void => {
    ev.preventDefault();
    gestureScale = null;
    glide.release("zoom");
    ensureGlideLoop();
  };

  // ── Multi-Touch Orbit & Pinch Zoom ──
  const readTouch = (e: TouchEvent) => {
    const contacts = Array.from(e.touches)
      .filter((c) => ownedTouches.has(c.identifier))
      .sort((a, b) => a.identifier - b.identifier);
    if (contacts.length === 0 || contacts.length > 2) return null;
    const [a, b = a] = contacts;
    return {
      ids: contacts.map((c) => c.identifier),
      x: (a!.clientX + b!.clientX) / 2,
      y: (a!.clientY + b!.clientY) / 2,
      distance: Math.hypot(a!.clientX - b!.clientX, a!.clientY - b!.clientY),
    };
  };

  const touchFrame = (movement: Partial<Omit<TouchSurfaceFrame, "fingers">> = {}) => {
    const frame: TouchSurfaceFrame = {
      fingers: ownedTouches.size,
      yaw: 0,
      pitch: 0,
      zoom: 1,
      ...movement,
    };
    touchGlide.frame(frame);
    ensureGlideLoop();
  };

  const onTouchStart = (e: TouchEvent): void => {
    if (e.defaultPrevented) return;
    let landed = false;
    for (const contact of Array.from(e.changedTouches)) {
      if (contact.target !== canvas || ownedTouches.has(contact.identifier)) continue;
      if (
        ownedTouches.size === 0 &&
        (options.isClickableAt?.(contact.clientX, contact.clientY, 0) ||
          isBabylonClickableAt(options.scene, canvas, contact.clientX, contact.clientY))
      ) {
        continue;
      }
      ownedTouches.add(contact.identifier);
      landed = true;
    }
    if (ownedTouches.size === 0) return;
    e.preventDefault();
    if (landed) touchFrame();
    gestureScale = null;
    touch = readTouch(e);
    touchWheelCooldownUntil = 0;
  };

  const onTouchMove = (e: TouchEvent): void => {
    if (ownedTouches.size === 0) return;
    e.preventDefault();
    const next = readTouch(e);
    if (
      touch &&
      next &&
      touch.ids.length === next.ids.length &&
      touch.ids.every((id, idx) => id === next.ids[idx]) &&
      gestureScale === null
    ) {
      const { yaw, pitch } = orbitDelta(touch.x - next.x, touch.y - next.y);
      const zoom =
        touch.ids.length === 2 && touch.distance > 0 && next.distance > 0
          ? zoomFactor(touch.distance / next.distance)
          : 1;
      touchFrame({ yaw, pitch, zoom });
      options.onUserOrbit?.();
    }
    touch = next;
  };

  const onTouchEnd = (e: TouchEvent): void => {
    if (ownedTouches.size === 0) return;
    e.preventDefault();
    let lifted = false;
    for (const contact of Array.from(e.changedTouches)) {
      lifted = ownedTouches.delete(contact.identifier) || lifted;
    }
    if (lifted) touchFrame();
    touch = readTouch(e);
    touchWheelCooldownUntil = performance.now() + 180;
    if (ownedTouches.size === 0) {
      options.onOrbitGestureEnd?.();
    }
  };

  const onKeyDown = (e: KeyboardEvent): void => {
    controlHeld = e.ctrlKey || e.key === "Control";
  };

  const onKeyUp = (e: KeyboardEvent): void => {
    controlHeld = e.ctrlKey;
  };

  const onWindowBlur = (): void => {
    finishPointerPress();
    ownedTouches.clear();
    touch = null;
    gestureScale = null;
    controlHeld = false;
    glide.cancel();
    touchGlide = makeTouchGlide();
  };

  const captureOpts: AddEventListenerOptions = { capture: true, passive: false };
  canvas.addEventListener("pointerdown", onPointerDown, captureOpts);
  owner.addEventListener("pointermove", onPointerMove, captureOpts);
  owner.addEventListener("pointerup", onPointerUp, captureOpts);
  owner.addEventListener("pointercancel", onPointerUp, captureOpts);
  canvas.addEventListener("lostpointercapture", onPointerUp, captureOpts);
  canvas.addEventListener("contextmenu", onContextMenu);
  canvas.addEventListener("wheel", onWheel, { passive: false });
  canvas.addEventListener("gesturestart", onGestureStart, { passive: false });
  canvas.addEventListener("gesturechange", onGestureChange, { passive: false });
  canvas.addEventListener("gestureend", onGestureEnd, { passive: false });
  canvas.addEventListener("touchstart", onTouchStart, { passive: false });
  canvas.addEventListener("touchmove", onTouchMove, { passive: false });
  canvas.addEventListener("touchend", onTouchEnd, { passive: false });
  canvas.addEventListener("touchcancel", onTouchEnd, { passive: false });
  owner.addEventListener("keydown", onKeyDown);
  owner.addEventListener("keyup", onKeyUp);
  owner.addEventListener("blur", onWindowBlur);

  return () => {
    if (swipeEndTimer !== null) clearTimeout(swipeEndTimer);
    if (rafId !== null) {
      owner.cancelAnimationFrame(rafId);
      rafId = null;
    }
    glide.cancel();
    canvas.removeEventListener("pointerdown", onPointerDown, captureOpts);
    owner.removeEventListener("pointermove", onPointerMove, captureOpts);
    owner.removeEventListener("pointerup", onPointerUp, captureOpts);
    owner.removeEventListener("pointercancel", onPointerUp, captureOpts);
    canvas.removeEventListener("lostpointercapture", onPointerUp, captureOpts);
    canvas.removeEventListener("contextmenu", onContextMenu);
    canvas.removeEventListener("wheel", onWheel);
    canvas.removeEventListener("gesturestart", onGestureStart);
    canvas.removeEventListener("gesturechange", onGestureChange);
    canvas.removeEventListener("gestureend", onGestureEnd);
    canvas.removeEventListener("touchstart", onTouchStart);
    canvas.removeEventListener("touchmove", onTouchMove);
    canvas.removeEventListener("touchend", onTouchEnd);
    canvas.removeEventListener("touchcancel", onTouchEnd);
    owner.removeEventListener("keydown", onKeyDown);
    owner.removeEventListener("keyup", onKeyUp);
    owner.removeEventListener("blur", onWindowBlur);
  };
}
