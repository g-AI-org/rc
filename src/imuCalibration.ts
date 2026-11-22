/**
 * Browser-native TypeScript IMU Calibration & HIL Benchmark Solvers.
 *
 * Zero Node/Python dependencies — runs directly in desktop/mobile browsers (and Node tests):
 * 1. 6-Face Accelerometer Ellipsoid Calibration (Levenberg-Marquardt 6-parameter bias + scale fit)
 *    Works on ANY >= 6 distinct static orientations (even tilted — no flat jig required).
 * 2. 3D IMU-to-CG Lever-Arm Calibration (zero-phase Butterworth + Savitzky-Golay + closed-form
 *    regularized least squares on pure-rotation maneuvers).
 * 3. Offline HIL Maneuver Benchmark Evaluator running the exact same `createWheeledRobotPhysics`
 *    6-DOF estimator over recorded 100 Hz telemetry.
 */

import {
  createWheeledRobotPhysics,
  ImuTelemetrySample,
  WheeledRobotPhysicsOptions,
} from "./imuPhysics";

export type Vec3Tuple = [number, number, number];

export interface ActiveImuCalibration {
  calibrated: boolean;
  accel6Face: boolean;
  gyroBiasDps: Vec3Tuple;
  accelBiasG: Vec3Tuple;
  accelScale: Vec3Tuple;
  leverArmM: Vec3Tuple;
}

export const DEFAULT_IMU_CALIBRATION: ActiveImuCalibration = {
  calibrated: false,
  accel6Face: false,
  gyroBiasDps: [0, 0, 0],
  accelBiasG: [0, 0, 0],
  accelScale: [1, 1, 1],
  leverArmM: [0, 0, 0],
};

export interface SixFaceCalibrationResult {
  biasG: Vec3Tuple;
  scale: Vec3Tuple;
  beforeRmsErrG: number;
  afterRmsErrG: number;
  beforeMaxErrG: number;
  afterMaxErrG: number;
  perFaceBeforeG: number[];
  perFaceAfterG: number[];
  axisSpanG: Vec3Tuple;
  wellConditioned: boolean;
  warning?: string;
}

export interface LeverArmCalibrationResult {
  /** CG -> IMU vector in robot body frame (+X Left, +Y Up, +Z Forward), meters. */
  leverArmM: Vec3Tuple;
  rmsBeforeMps2: number;
  rmsAfterMps2: number;
  excitationSamples: number;
  minAxisExcitation: number;
  quality: "good" | "low_excitation";
  warning?: string;
}

export interface StillnessWindowStats {
  ok: boolean;
  sampleCount: number;
  meanAccelG: Vec3Tuple;
  meanGyroDps: Vec3Tuple;
  accelStdG: number;
  gyroSpreadDps: number;
  reason?: string;
}

export interface HilManeuverPreset {
  id: string;
  label: string;
  durationSec: number;
  instruction: string;
  expectedMetric: string;
}

export const HIL_MANEUVER_PRESETS: readonly HilManeuverPreset[] = [
  {
    id: "01_still",
    label: "Still (6s)",
    durationSec: 6,
    instruction: "Set the robot down and do not touch it until the second beep.",
    expectedMetric: "Rest > 90%, Drift < 0.5 cm, Yaw < 0.5°",
  },
  {
    id: "02_rotate_cg",
    label: "Rotate CG (8s)",
    durationSec: 8,
    instruction: "Hold around the wheels/CG and pitch, roll & yaw smoothly in place (minimal translation).",
    expectedMetric: "Solves CG lever-arm & checks translation < 3 cm",
  },
  {
    id: "03_spin360",
    label: "Spin 360° (6s)",
    durationSec: 6,
    instruction: "Spin the robot flat on the desk by one full 360° turn and stop.",
    expectedMetric: "Yaw ≈ ±360°, Drift < 3 cm",
  },
  {
    id: "04_slide30_back",
    label: "Slide 30cm & Back (6s)",
    durationSec: 6,
    instruction: "Slide right ~30 cm smoothly, pause 1s, slide back to exact start.",
    expectedMetric: "Peak ≈ 30 cm, Return < 5 cm",
  },
  {
    id: "05_lift25_back",
    label: "Lift 25cm & Back (6s)",
    durationSec: 6,
    instruction: "Lift straight up ~25 cm, pause 1s, set gently back down on start.",
    expectedMetric: "Peak Y ≈ 25 cm, Return < 4 cm",
  },
];

export interface HilCapture {
  name: string;
  recordedAt: string;
  durationSec: number;
  robotIp?: string;
  calibration?: ActiveImuCalibration;
  samples: ImuTelemetrySample[];
}

export interface HilScorecard {
  sampleCount: number;
  durationSec: number;
  restPercent: number;
  accelNormMeanG: number;
  accelNormStdG: number;
  gyroMeanDps: Vec3Tuple;
  endPosCm: Vec3Tuple;
  maxDispCm: number;
  maxLiftCm: number;
  returnErrCm: number;
  yawDeltaDeg: number;
  maxTiltDeg: number;
}

const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;
const G_MS2 = 9.80665;

function parseTuple3(line: string): Vec3Tuple | null {
  const m = line.match(/\(\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*\)/);
  if (!m) return null;
  const x = Number(m[1]);
  const y = Number(m[2]);
  const z = Number(m[3]);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) return null;
  return [x, y, z];
}

/**
 * Parses output from the firmware `imu` console command into structured calibration settings.
 */
export function parseImuStatusOutput(output: string): ActiveImuCalibration {
  const res: ActiveImuCalibration = {
    calibrated: DEFAULT_IMU_CALIBRATION.calibrated,
    accel6Face: DEFAULT_IMU_CALIBRATION.accel6Face,
    gyroBiasDps: [...DEFAULT_IMU_CALIBRATION.gyroBiasDps],
    accelBiasG: [...DEFAULT_IMU_CALIBRATION.accelBiasG],
    accelScale: [...DEFAULT_IMU_CALIBRATION.accelScale],
    leverArmM: [...DEFAULT_IMU_CALIBRATION.leverArmM],
  };
  for (const rawLine of output.split(/\r?\n/)) {
    const line = rawLine.trim();
    if (line.startsWith("calibrated")) {
      res.calibrated = /\byes\b/i.test(line);
    } else if (line.startsWith("accel_6face")) {
      res.accel6Face = /\byes\b/i.test(line);
    } else if (line.startsWith("gyro_bias")) {
      const t = parseTuple3(line);
      if (t) res.gyroBiasDps = t;
    } else if (line.startsWith("accel_bias")) {
      const t = parseTuple3(line);
      if (t) res.accelBiasG = t;
    } else if (line.startsWith("accel_scale")) {
      const t = parseTuple3(line);
      if (t) res.accelScale = t;
    } else if (line.startsWith("lever_arm_m")) {
      const t = parseTuple3(line);
      if (t) res.leverArmM = t;
    }
  }
  return res;
}

/**
 * Recovers raw uncalibrated MPU6886 accel (g) from calibrated telemetry using the active
 * flash parameters: a_cal = (a_raw - bias) * scale  =>  a_raw = a_cal / scale + bias.
 */
export function unapplyAccelCalibration(
  calAccelG: Vec3Tuple,
  activeCal: ActiveImuCalibration
): Vec3Tuple {
  const [ax, ay, az] = calAccelG;
  const [bx, by, bz] = activeCal.accelBiasG;
  const [sx, sy, sz] = activeCal.accelScale;
  return [
    ax / (Math.abs(sx) > 1e-4 ? sx : 1) + bx,
    ay / (Math.abs(sy) > 1e-4 ? sy : 1) + by,
    az / (Math.abs(sz) > 1e-4 ? sz : 1) + bz,
  ];
}

/**
 * Validates that a telemetry slice is stationary and returns its mean accel (g) and gyro (dps).
 */
export function extractStillnessSample(
  samples: readonly ImuTelemetrySample[],
  maxGyroSpreadDps = 18,
  maxAccelStdG = 0.045
): StillnessWindowStats {
  const n = samples.length;
  if (n < 20) {
    return {
      ok: false,
      sampleCount: n,
      meanAccelG: [0, 0, 0],
      meanGyroDps: [0, 0, 0],
      accelStdG: 0,
      gyroSpreadDps: 0,
      reason: `Need at least 20 samples (got ${n}).`,
    };
  }

  let sax = 0, say = 0, saz = 0;
  let sgx = 0, sgy = 0, sgz = 0;
  let minGx = Infinity, maxGx = -Infinity;
  let minGy = Infinity, maxGy = -Infinity;
  let minGz = Infinity, maxGz = -Infinity;

  for (const s of samples) {
    const ax = s.accelXG ?? 0;
    const ay = s.accelYG ?? -1;
    const az = s.accelZG ?? 0;
    const gx = s.gyroXDps ?? 0;
    const gy = s.gyroYDps ?? 0;
    const gz = s.gyroZDps ?? 0;
    sax += ax; say += ay; saz += az;
    sgx += gx; sgy += gy; sgz += gz;
    if (gx < minGx) minGx = gx; if (gx > maxGx) maxGx = gx;
    if (gy < minGy) minGy = gy; if (gy > maxGy) maxGy = gy;
    if (gz < minGz) minGz = gz; if (gz > maxGz) maxGz = gz;
  }

  const max = sax / n, may = say / n, maz = saz / n;
  const mgx = sgx / n, mgy = sgy / n, mgz = sgz / n;

  let varA = 0;
  for (const s of samples) {
    const dx = (s.accelXG ?? 0) - max;
    const dy = (s.accelYG ?? -1) - may;
    const dz = (s.accelZG ?? 0) - maz;
    varA += dx * dx + dy * dy + dz * dz;
  }
  const accelStdG = Math.sqrt(varA / n);
  const gyroSpreadDps = Math.max(maxGx - minGx, maxGy - minGy, maxGz - minGz);

  if (gyroSpreadDps > maxGyroSpreadDps || accelStdG > maxAccelStdG) {
    return {
      ok: false,
      sampleCount: n,
      meanAccelG: [max, may, maz],
      meanGyroDps: [mgx, mgy, mgz],
      accelStdG,
      gyroSpreadDps,
      reason: `Robot moved (gyro spread ${gyroSpreadDps.toFixed(1)} dps, accel std ${accelStdG.toFixed(3)} g). Hold still and retry.`,
    };
  }

  return {
    ok: true,
    sampleCount: n,
    meanAccelG: [max, may, maz],
    meanGyroDps: [mgx, mgy, mgz],
    accelStdG,
    gyroSpreadDps,
  };
}

/** Solves N x N linear system A * x = b via Gaussian elimination with partial pivoting. */
function solveLinearSystem(A: number[][], b: number[]): number[] | null {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]!]);

  for (let col = 0; col < n; col++) {
    let pivot = col;
    let best = Math.abs(M[col]![col]!);
    for (let r = col + 1; r < n; r++) {
      const v = Math.abs(M[r]![col]!);
      if (v > best) {
        best = v;
        pivot = r;
      }
    }
    if (best < 1e-12) return null;
    if (pivot !== col) {
      const tmp = M[col]!;
      M[col] = M[pivot]!;
      M[pivot] = tmp;
    }
    const diag = M[col]![col]!;
    for (let j = col; j <= n; j++) M[col]![j]! /= diag;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const factor = M[r]![col]!;
      if (factor === 0) continue;
      for (let j = col; j <= n; j++) {
        M[r]![j]! -= factor * M[col]![j]!;
      }
    }
  }
  return M.map((row) => row[n]!);
}

/**
 * 6-Face Accelerometer Ellipsoid Calibration (Levenberg-Marquardt).
 *
 * Solves bias b = (bx, by, bz) and scale s = (sx, sy, sz) such that
 *   ||(a_i - b) * s|| == 1.000 g
 * across N >= 6 distinct static orientations (raw sensor g).
 */
export function solveSixFaceAccelEllipsoid(
  faceMeansRawG: readonly Vec3Tuple[]
): SixFaceCalibrationResult {
  const n = faceMeansRawG.length;
  if (n < 6) {
    throw new Error(`Need at least 6 distinct static orientations (got ${n}).`);
  }

  // Axis span diagnostics + smart initial guess from bounding box
  let minX = Infinity, maxX = -Infinity;
  let minY = Infinity, maxY = -Infinity;
  let minZ = Infinity, maxZ = -Infinity;
  for (const [x, y, z] of faceMeansRawG) {
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
  }
  const spanX = maxX - minX;
  const spanY = maxY - minY;
  const spanZ = maxZ - minZ;

  // Params: [bx, by, bz, sx, sy, sz]
  let p = [
    0.5 * (maxX + minX),
    0.5 * (maxY + minY),
    0.5 * (maxZ + minZ),
    spanX > 0.5 ? Math.max(0.85, Math.min(1.2, 2 / spanX)) : 1,
    spanY > 0.5 ? Math.max(0.85, Math.min(1.2, 2 / spanY)) : 1,
    spanZ > 0.5 ? Math.max(0.85, Math.min(1.2, 2 / spanZ)) : 1,
  ];

  const evalCost = (theta: number[]) => {
    let sumSq = 0;
    for (const [ax, ay, az] of faceMeansRawG) {
      const ux = theta[3]! * (ax - theta[0]!);
      const uy = theta[4]! * (ay - theta[1]!);
      const uz = theta[5]! * (az - theta[2]!);
      const r = Math.hypot(ux, uy, uz) - 1;
      sumSq += r * r;
    }
    return sumSq;
  };

  let lambda = 1e-3;
  let cost = evalCost(p);

  for (let iter = 0; iter < 40; iter++) {
    const H = Array.from({ length: 6 }, () => new Array<number>(6).fill(0));
    const g = new Array<number>(6).fill(0);

    for (const [ax, ay, az] of faceMeansRawG) {
      const dx = ax - p[0]!;
      const dy = ay - p[1]!;
      const dz = az - p[2]!;
      const ux = p[3]! * dx;
      const uy = p[4]! * dy;
      const uz = p[5]! * dz;
      const norm = Math.max(1e-9, Math.hypot(ux, uy, uz));
      const r = norm - 1;
      const J = [
        -(ux / norm) * p[3]!,
        -(uy / norm) * p[4]!,
        -(uz / norm) * p[5]!,
        (ux / norm) * dx,
        (uy / norm) * dy,
        (uz / norm) * dz,
      ];
      for (let i = 0; i < 6; i++) {
        g[i]! -= J[i]! * r;
        for (let j = 0; j < 6; j++) {
          H[i]![j]! += J[i]! * J[j]!;
        }
      }
    }

    for (let i = 0; i < 6; i++) {
      H[i]![i]! += lambda * (1 + H[i]![i]!);
    }

    const step = solveLinearSystem(H, g);
    if (!step) {
      lambda *= 10;
      continue;
    }

    const cand = p.map((v, i) => v + step[i]!);
    // Keep physically plausible bounds during search
    cand[0] = Math.max(-0.45, Math.min(0.45, cand[0]!));
    cand[1] = Math.max(-0.45, Math.min(0.45, cand[1]!));
    cand[2] = Math.max(-0.45, Math.min(0.45, cand[2]!));
    cand[3] = Math.max(0.80, Math.min(1.25, cand[3]!));
    cand[4] = Math.max(0.80, Math.min(1.25, cand[4]!));
    cand[5] = Math.max(0.80, Math.min(1.25, cand[5]!));

    const candCost = evalCost(cand);
    if (candCost < cost) {
      const improvement = cost - candCost;
      p = cand;
      cost = candCost;
      lambda = Math.max(1e-7, lambda * 0.3);
      if (improvement < 1e-13) break;
    } else {
      lambda = Math.min(1e6, lambda * 5);
    }
  }

  const perFaceBeforeG = faceMeansRawG.map(([x, y, z]) => Math.hypot(x, y, z));
  const perFaceAfterG = faceMeansRawG.map(([x, y, z]) =>
    Math.hypot(p[3]! * (x - p[0]!), p[4]! * (y - p[1]!), p[5]! * (z - p[2]!))
  );

  const rms = (arr: number[]) =>
    Math.sqrt(arr.reduce((acc, v) => acc + (v - 1) * (v - 1), 0) / arr.length);
  const maxErr = (arr: number[]) =>
    arr.reduce((acc, v) => Math.max(acc, Math.abs(v - 1)), 0);

  const wellConditioned = spanX >= 1.2 && spanY >= 1.2 && spanZ >= 1.2;
  const warning = !wellConditioned
    ? `Low axis coverage (spans X=${spanX.toFixed(2)}, Y=${spanY.toFixed(2)}, Z=${spanZ.toFixed(2)} g). Make sure you capture opposite faces on all 3 axes.`
    : undefined;

  return {
    biasG: [p[0]!, p[1]!, p[2]!],
    scale: [p[3]!, p[4]!, p[5]!],
    beforeRmsErrG: rms(perFaceBeforeG),
    afterRmsErrG: rms(perFaceAfterG),
    beforeMaxErrG: maxErr(perFaceBeforeG),
    afterMaxErrG: maxErr(perFaceAfterG),
    perFaceBeforeG,
    perFaceAfterG,
    axisSpanG: [spanX, spanY, spanZ],
    wellConditioned,
    warning,
  };
}

/** Zero-phase forward-backward 2nd-order Butterworth low-pass filter for offline calibration. */
function zeroPhaseLowPass3(data: Vec3Tuple[], fcHz: number, fsHz: number): Vec3Tuple[] {
  const n = data.length;
  if (n < 6) return data.map((v) => [...v] as Vec3Tuple);

  const k = Math.tan((Math.PI * fcHz) / fsHz);
  const norm = 1 / (1 + Math.SQRT2 * k + k * k);
  const b0 = k * k * norm;
  const b1 = 2 * b0;
  const b2 = b0;
  const a1 = 2 * (k * k - 1) * norm;
  const a2 = (1 - Math.SQRT2 * k + k * k) * norm;

  const pass = (inp: Vec3Tuple[]): Vec3Tuple[] => {
    const out: Vec3Tuple[] = new Array(n);
    let x1 = inp[0]!, x2 = inp[0]!, y1 = inp[0]!, y2 = inp[0]!;
    for (let i = 0; i < n; i++) {
      const x = inp[i]!;
      const y: Vec3Tuple = [
        b0 * x[0] + b1 * x1[0] + b2 * x2[0] - a1 * y1[0] - a2 * y2[0],
        b0 * x[1] + b1 * x1[1] + b2 * x2[1] - a1 * y1[1] - a2 * y2[1],
        b0 * x[2] + b1 * x1[2] + b2 * x2[2] - a1 * y1[2] - a2 * y2[2],
      ];
      out[i] = y;
      x2 = x1; x1 = x; y2 = y1; y1 = y;
    }
    return out;
  };

  const fwd = pass(data);
  const rev = pass([...fwd].reverse()).reverse();
  return rev;
}

/**
 * Closed-Form 3D IMU-to-CG Lever-Arm Solver from a pure-rotation capture.
 *
 * When rotating the robot in place around its CG / wheel axle:
 *   a_imu_body - g_body = ([alpha]_x + [omega]_x^2) * r_body
 * where r_body = (rx, ry, rz) is the vector from CG to IMU in robot body frame
 * (+X Left, +Y Up, +Z Forward), in meters.
 */
export function solveLeverArmFromRotation(
  samples: readonly ImuTelemetrySample[],
  dtSec = 0.01
): LeverArmCalibrationResult {
  const n = samples.length;
  if (n < 50) {
    throw new Error(`Need at least 50 samples (~0.5s) of rotation data (got ${n}).`);
  }

  const fs = 1 / dtSec;

  // Map samples into Right-Handed Body Frame B (+X Left, +Y Up, +Z Forward)
  const aBodyMs2: Vec3Tuple[] = new Array(n);
  const wBodyRad: Vec3Tuple[] = new Array(n);
  for (let i = 0; i < n; i++) {
    const s = samples[i]!;
    aBodyMs2[i] = [
      -(s.accelXG ?? 0) * G_MS2,
      -(s.accelYG ?? -1) * G_MS2,
      (s.accelZG ?? 0) * G_MS2,
    ];
    wBodyRad[i] = [
      -(s.gyroXDps ?? s.pitchDps ?? 0) * DEG_TO_RAD,
      -(s.gyroYDps ?? 0) * DEG_TO_RAD,
      (s.gyroZDps ?? 0) * DEG_TO_RAD,
    ];
  }

  // Zero-phase 8 Hz low-pass filter on angular velocity & compute 5-pt centered angular acceleration
  const wFilt = zeroPhaseLowPass3(wBodyRad, 8.0, fs);
  const alphaBody: Vec3Tuple[] = new Array(n);
  for (let i = 0; i < n; i++) {
    if (i >= 2 && i < n - 2) {
      const wm2 = wFilt[i - 2]!, wm1 = wFilt[i - 1]!, wp1 = wFilt[i + 1]!, wp2 = wFilt[i + 2]!;
      const inv12dt = 1 / (12 * dtSec);
      alphaBody[i] = [
        (wm2[0] - 8 * wm1[0] + 8 * wp1[0] - wp2[0]) * inv12dt,
        (wm2[1] - 8 * wm1[1] + 8 * wp1[1] - wp2[1]) * inv12dt,
        (wm2[2] - 8 * wm1[2] + 8 * wp1[2] - wp2[2]) * inv12dt,
      ];
    } else {
      alphaBody[i] = [0, 0, 0];
    }
  }

  // Quaternion helpers (zero render-smoothing lag!)
  type Q4 = [number, number, number, number]; // [w, x, y, z]
  const qMul = (a: Q4, b: Q4): Q4 => [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
  const qNorm = (q: Q4): Q4 => {
    const l = Math.hypot(q[0], q[1], q[2], q[3]);
    return l > 1e-12 ? [q[0] / l, q[1] / l, q[2] / l, q[3] / l] : [1, 0, 0, 0];
  };
  const qRot = (q: Q4, v: Vec3Tuple): Vec3Tuple => {
    const [w, x, y, z] = q;
    const tx = 2 * (y * v[2] - z * v[1]);
    const ty = 2 * (z * v[0] - x * v[2]);
    const tz = 2 * (x * v[1] - y * v[0]);
    return [
      v[0] + w * tx + (y * tz - z * ty),
      v[1] + w * ty + (z * tx - x * tz),
      v[2] + w * tz + (x * ty - y * tx),
    ];
  };
  const qAxisAngle = (ax: number, ay: number, az: number, ang: number): Q4 => {
    const h = 0.5 * ang;
    const s = Math.sin(h);
    return [Math.cos(h), ax * s, ay * s, az * s];
  };
  const qShortestArc = (from: Vec3Tuple, to: Vec3Tuple): Q4 => {
    const dot = Math.max(-1, Math.min(1, from[0] * to[0] + from[1] * to[1] + from[2] * to[2]));
    if (dot < -0.99999) {
      const o: Vec3Tuple = Math.abs(from[0]) < 0.9 ? [1, 0, 0] : [0, 0, 1];
      const cx = from[1] * o[2] - from[2] * o[1];
      const cy = from[2] * o[0] - from[0] * o[2];
      const cz = from[0] * o[1] - from[1] * o[0];
      const l = Math.hypot(cx, cy, cz);
      return l > 1e-9 ? [0, cx / l, cy / l, cz / l] : [0, 1, 0, 0];
    }
    return qNorm([
      1 + dot,
      from[1] * to[2] - from[2] * to[1],
      from[2] * to[0] - from[0] * to[2],
      from[0] * to[1] - from[1] * to[0],
    ]);
  };

  // Two-pass unlagged orientation + lever-arm refinement:
  // Pass 1 estimates r with raw VQF gravity; Pass 2 subtracts lever-arm accel before estimating
  // gravity so centripetal/tangential accel cannot bias tilt during fast rotations.
  let rEst: Vec3Tuple = [0, 0, 0];
  let rmsBeforeMps2 = 0;
  let rmsAfterMps2 = 0;
  let excitationSamples = 0;
  let minAxisExcitation = 0;

  for (let pass = 0; pass < 2; pass++) {
    // Integrate exact midpoint SO(3) orientation qGyr(t) from raw gyro
    const qGyrArr: Q4[] = new Array(n);
    const aInertArr: Vec3Tuple[] = new Array(n);
    let qGyr: Q4 = [1, 0, 0, 0];
    for (let i = 0; i < n; i++) {
      const w = wBodyRad[i]!;
      const wn = Math.hypot(w[0], w[1], w[2]);
      if (wn > 1e-8) {
        qGyr = qNorm(qMul(qGyr, qAxisAngle(w[0] / wn, w[1] / wn, w[2] / wn, wn * dtSec)));
      }
      qGyrArr[i] = qGyr;

      // Subtract current lever-arm estimate to recover clean gravity in inertial frame
      const ab = aBodyMs2[i]!;
      let acg: Vec3Tuple = ab;
      if (pass > 0) {
        const [wx, wy, wz] = wFilt[i]!;
        const [ax, ay, az] = alphaBody[i]!;
        const [rx, ry, rz] = rEst;
        const wdr = wx * rx + wy * ry + wz * rz;
        const w2 = wx * wx + wy * wy + wz * wz;
        acg = [
          ab[0] - (ay * rz - az * ry + wdr * wx - w2 * rx),
          ab[1] - (az * rx - ax * rz + wdr * wy - w2 * ry),
          ab[2] - (ax * ry - ay * rx + wdr * wz - w2 * rz),
        ];
      }
      aInertArr[i] = qRot(qGyr, acg);
    }

    // Zero-phase very-low-pass (0.35 Hz) inertial gravity vector => exact world-up direction
    // with ZERO phase lag and minimal drift over the 8s capture!
    const gInertLp = zeroPhaseLowPass3(aInertArr, 0.35, fs);

    const yRaw: Vec3Tuple[] = new Array(n);
    for (let i = 0; i < n; i++) {
      const gi = gInertLp[i]!;
      const gn = Math.hypot(gi[0], gi[1], gi[2]);
      const gUnitI: Vec3Tuple = gn > 1 ? [gi[0] / gn, gi[1] / gn, gi[2] / gn] : [0, 1, 0];
      const qCorr = qShortestArc(gUnitI, [0, 1, 0]);
      const qFused = qNorm(qMul(qCorr, qGyrArr[i]!));
      const gB = qRot([qFused[0], -qFused[1], -qFused[2], -qFused[3]], [0, G_MS2, 0]);
      const ab = aBodyMs2[i]!;
      yRaw[i] = [ab[0] - gB[0], ab[1] - gB[1], ab[2] - gB[2]];
    }

    const yFilt = zeroPhaseLowPass3(yRaw, 8.0, fs);

    const H = [
      [0, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
    ];
    const rhs = [0, 0, 0];
    const rowsM: number[][][] = [];
    const rowsY: Vec3Tuple[] = [];

    for (let i = 4; i < n - 4; i++) {
      const [wx, wy, wz] = wFilt[i]!;
      const [ax, ay, az] = alphaBody[i]!;
      const wNorm = Math.hypot(wx, wy, wz);
      const aNorm = Math.hypot(ax, ay, az);
      if (wNorm < 0.35 && aNorm < 2.0) continue;

      const M = [
        [-(wy * wy + wz * wz), wx * wy - az, wx * wz + ay],
        [wx * wy + az, -(wx * wx + wz * wz), wy * wz - ax],
        [wx * wz - ay, wy * wz + ax, -(wx * wx + wy * wy)],
      ];
      const y = yFilt[i]!;
      rowsM.push(M);
      rowsY.push(y);

      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) {
          H[r]![c]! += M[0]![r]! * M[0]![c]! + M[1]![r]! * M[1]![c]! + M[2]![r]! * M[2]![c]!;
        }
        rhs[r]! += M[0]![r]! * y[0] + M[1]![r]! * y[1] + M[2]![r]! * y[2];
      }
    }

    excitationSamples = rowsM.length;
    minAxisExcitation = Math.min(H[0]![0]!, H[1]![1]!, H[2]![2]!);

    if (excitationSamples < 25) {
      return {
        leverArmM: [0, 0, 0],
        rmsBeforeMps2: 0,
        rmsAfterMps2: 0,
        excitationSamples,
        minAxisExcitation,
        quality: "low_excitation",
        warning: "Not enough rotational motion detected. Pitch, roll, and yaw briskly around the CG/axle.",
      };
    }

    const lambda = 0.5;
    H[0]![0]! += lambda;
    H[1]![1]! += lambda;
    H[2]![2]! += lambda;

    const solved = solveLinearSystem(H, rhs) ?? [0, 0, 0];
    rEst = [
      Math.max(-0.35, Math.min(0.35, solved[0]!)),
      Math.max(-0.35, Math.min(0.35, solved[1]!)),
      Math.max(-0.35, Math.min(0.35, solved[2]!)),
    ];

    let sumSqBefore = 0;
    let sumSqAfter = 0;
    for (let i = 0; i < rowsM.length; i++) {
      const M = rowsM[i]!;
      const y = rowsY[i]!;
      const pred0 = M[0]![0]! * rEst[0] + M[0]![1]! * rEst[1] + M[0]![2]! * rEst[2];
      const pred1 = M[1]![0]! * rEst[0] + M[1]![1]! * rEst[1] + M[1]![2]! * rEst[2];
      const pred2 = M[2]![0]! * rEst[0] + M[2]![1]! * rEst[1] + M[2]![2]! * rEst[2];
      sumSqBefore += y[0] * y[0] + y[1] * y[1] + y[2] * y[2];
      const e0 = y[0] - pred0;
      const e1 = y[1] - pred1;
      const e2 = y[2] - pred2;
      sumSqAfter += e0 * e0 + e1 * e1 + e2 * e2;
    }
    rmsBeforeMps2 = Math.sqrt(sumSqBefore / (rowsM.length * 3));
    rmsAfterMps2 = Math.sqrt(sumSqAfter / (rowsM.length * 3));
  }

  const good = minAxisExcitation >= 800 && excitationSamples >= 60;
  return {
    leverArmM: rEst,
    rmsBeforeMps2,
    rmsAfterMps2,
    excitationSamples,
    minAxisExcitation,
    quality: good ? "good" : "low_excitation",
    warning: good
      ? undefined
      : "Some rotation axes had low excitation — try pitching, rolling, and yawing briskly around the wheels/CG.",
  };
}

/**
 * Runs the exact `@g-ai/rc` 6-DOF physics estimator over a recorded HIL capture in the browser
 * and returns a quantitative benchmark scorecard.
 */
export function evaluateHilCapture(
  samples: readonly ImuTelemetrySample[],
  options: WheeledRobotPhysicsOptions = {}
): HilScorecard {
  const n = samples.length;
  if (n === 0) {
    return {
      sampleCount: 0,
      durationSec: 0,
      restPercent: 0,
      accelNormMeanG: 0,
      accelNormStdG: 0,
      gyroMeanDps: [0, 0, 0],
      endPosCm: [0, 0, 0],
      maxDispCm: 0,
      maxLiftCm: 0,
      returnErrCm: 0,
      yawDeltaDeg: 0,
      maxTiltDeg: 0,
    };
  }

  const sim = createWheeledRobotPhysics(options);
  let restCount = 0;
  let sumNorm = 0;
  let sumNorm2 = 0;
  let sgx = 0, sgy = 0, sgz = 0;

  let startX = 0, startY = 0, startZ = 0;
  let startYaw = 0;
  let prevYaw = 0;
  let unwrappedYaw = 0;
  let maxDisp = 0;
  let maxLift = 0;
  let maxTiltDeg = 0;
  let endX = 0, endY = 0, endZ = 0;

  for (let i = 0; i < n; i++) {
    const s = samples[i]!;
    const ax = s.accelXG ?? 0;
    const ay = s.accelYG ?? -1;
    const az = s.accelZG ?? 0;
    const aNorm = Math.hypot(ax, ay, az);
    sumNorm += aNorm;
    sumNorm2 += aNorm * aNorm;
    sgx += s.gyroXDps ?? 0;
    sgy += s.gyroYDps ?? 0;
    sgz += s.gyroZDps ?? 0;

    sim.setLatestSample(s);
    const st = sim.step(0.01);

    if (st.isResting) restCount++;

    if (i === 0) {
      startX = st.posX;
      startY = st.posY;
      startZ = st.posZ;
      startYaw = st.yawRad;
      prevYaw = st.yawRad;
      unwrappedYaw = st.yawRad;
    } else {
      let dyaw = st.yawRad - prevYaw;
      if (dyaw > Math.PI) dyaw -= 2 * Math.PI;
      if (dyaw < -Math.PI) dyaw += 2 * Math.PI;
      unwrappedYaw += dyaw;
      prevYaw = st.yawRad;
    }

    const dx = st.posX - startX;
    const dy = st.posY - startY;
    const dz = st.posZ - startZ;
    const disp = Math.hypot(dx, dy, dz);
    if (disp > maxDisp) maxDisp = disp;
    if (dy > maxLift) maxLift = dy;
    const tilt = Math.hypot(st.pitchRad * RAD_TO_DEG, st.rollRad * RAD_TO_DEG);
    if (tilt > maxTiltDeg) maxTiltDeg = tilt;

    endX = dx;
    endY = dy;
    endZ = dz;
  }

  const meanNorm = sumNorm / n;
  const stdNorm = Math.sqrt(Math.max(0, sumNorm2 / n - meanNorm * meanNorm));

  return {
    sampleCount: n,
    durationSec: n * 0.01,
    restPercent: (100 * restCount) / n,
    accelNormMeanG: meanNorm,
    accelNormStdG: stdNorm,
    gyroMeanDps: [sgx / n, sgy / n, sgz / n],
    endPosCm: [endX * 100, endY * 100, endZ * 100],
    maxDispCm: maxDisp * 100,
    maxLiftCm: maxLift * 100,
    returnErrCm: Math.hypot(endX, endY, endZ) * 100,
    yawDeltaDeg: (unwrappedYaw - startYaw) * RAD_TO_DEG,
    maxTiltDeg,
  };
}
