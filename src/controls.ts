/**
 * Shared joystick math, deadzone, expo curves, and floating-origin touch surface helpers.
 */
export interface StickVector {
  x: number; // -1 .. 1
  y: number; // -1 .. 1
}

export function clampUnit(v: number): number {
  return Math.max(-1, Math.min(1, v));
}

export function applyDeadzoneAndExpo(value: number, deadzone = 0.04, expo = 0.25): number {
  const abs = Math.abs(value);
  if (abs <= deadzone) return 0;
  const sign = Math.sign(value);
  const norm = (abs - deadzone) / (1 - deadzone);
  const curved = (1 - expo) * norm + expo * norm * norm * norm;
  return clampUnit(sign * curved);
}

export function computeStickVector(
  originX: number,
  originY: number,
  currentX: number,
  currentY: number,
  radiusPx: number,
  deadzone = 0.04,
  expo = 0.2
): StickVector {
  if (radiusPx <= 0) return { x: 0, y: 0 };
  const dx = (currentX - originX) / radiusPx;
  const dy = (originY - currentY) / radiusPx; // Up is positive Y
  const mag = Math.hypot(dx, dy);
  const scale = mag > 1 ? 1 / mag : 1;
  return {
    x: applyDeadzoneAndExpo(dx * scale, deadzone, expo),
    y: applyDeadzoneAndExpo(dy * scale, deadzone, expo),
  };
}
