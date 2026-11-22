/**
 * 2026 SOTA 6-DOF IMU Sensor Fusion (VQF-inspired decoupled quaternion filter)
 * & 3-Axis Linear Inertial + Wheeled Robot Physics Integrator.
 *
 * Key properties:
 * 1. Exact SO(3) exponential-map quaternion integration — zero gimbal lock at ANY 3D pose
 *    (standing upright, lying flat on back/face, rolled 90 deg, upside down, spinning flat on a table).
 * 2. VQF Inertial-frame 2nd-order Butterworth low-pass gravity alignment + Gaussian norm trust
 *    gating — linear hand motions & wheel acceleration do NOT corrupt tilt, while gravity
 *    cleanly eliminates pitch/roll drift without touching world yaw.
 * 3. Rest detection + online residual gyro bias adaptation — eliminates thermal yaw drift when still.
 * 4. 3-axis world linear motion (X, Y, Z) via gravity-compensated inertial acceleration,
 *    rotation-aware lever-arm deadband, midpoint integration, ZUPT (Zero-Velocity Updates),
 *    wheel odometry fusion, and exact rotated bounding-box ground contact.
 */

export interface ImuTelemetrySample {
  tick: number;
  state: string;
  motorL: number;
  motorR: number;
  pitchDeg: number;
  pitchDps: number;
  accelXG?: number;
  accelYG?: number;
  accelZG?: number;
  gyroXDps?: number;
  gyroYDps?: number;
  gyroZDps?: number;
  yawDeg?: number;
}

export interface QuatXYZW {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Vec3XYZ {
  x: number;
  y: number;
  z: number;
}

export interface RobotKinematicState {
  /** Unit orientation quaternion in Babylon.js left-handed world coordinates (+X Right, +Y Up, +Z Forward). */
  quat: QuatXYZW;
  /** Pitch angle in radians (positive = leaning back / LCD facing up). */
  pitchRad: number;
  /** Pitch angular velocity in rad/s. */
  pitchRateRadPerSec: number;
  /** Roll angle in radians (positive = leaning right). */
  rollRad: number;
  /** Roll angular velocity in rad/s. */
  rollRateRadPerSec: number;
  /** World yaw heading in radians (positive = CCW / turning left looking from above). */
  yawRad: number;
  /** Yaw angular velocity in rad/s. */
  yawRateRadPerSec: number;
  /** Left wheel spin angle in radians. */
  wheelAngleL: number;
  /** Right wheel spin angle in radians. */
  wheelAngleR: number;
  /** Filtered left wheel angular velocity in rad/s. */
  wheelOmegaL: number;
  /** Filtered right wheel angular velocity in rad/s. */
  wheelOmegaR: number;
  /** World X position in meters (+X Right in Babylon). */
  posX: number;
  /** World Y position in meters (+Y Up in Babylon, >= floorY). */
  posY: number;
  /** World Z position in meters (+Z Forward in Babylon). */
  posZ: number;
  /** Alias for posY kept for backward compatibility. */
  axleHeightY: number;
  /** Minimum non-clipping axle height above ground (y = 0) for the current 3D orientation. */
  floorY: number;
  /** World linear velocity in m/s (Babylon coordinates). */
  vel: Vec3XYZ;
  /** Gravity-compensated world linear acceleration in m/s^2 (Babylon coordinates). */
  linearAccel: Vec3XYZ;
  /** True when the VQF rest detector sees the robot stationary (ZUPT + online bias active). */
  isResting: boolean;
}

export interface WheeledRobotPhysicsOptions {
  /** Wheel radius in meters (default 0.021 m = 21 mm for BalaC). */
  wheelRadiusM?: number;
  /** Wheel track width in meters (default 0.068 m = 68 mm for BalaC). */
  trackWidthM?: number;
  /** Maximum wheel angular speed at full motor command 127 (default 14 rad/s). */
  maxWheelOmegaRadPerSec?: number;
  /** Wheel motor first-order time constant in seconds (default 0.035 s). */
  motorTauSec?: number;
  /** Arena half-extent in meters for X/Z and ceiling for Y (default 1.2 m). */
  arenaLimitM?: number;
  /** Enable 3-axis IMU linear translation integration (default true). */
  enableLinearMotion?: boolean;
  /** CG->IMU lever-arm vector in robot body frame (+X Left, +Y Up, +Z Forward), meters. */
  leverArmM?: readonly [number, number, number];
}

const DEG_TO_RAD = Math.PI / 180;
const G_MS2 = 9.80665;

interface QuatWXYZ {
  w: number;
  x: number;
  y: number;
  z: number;
}

function qMul(a: QuatWXYZ, b: QuatWXYZ): QuatWXYZ {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

function qNorm(q: QuatWXYZ): QuatWXYZ {
  const n = Math.hypot(q.w, q.x, q.y, q.z);
  if (n < 1e-12) return { w: 1, x: 0, y: 0, z: 0 };
  const inv = 1 / n;
  return { w: q.w * inv, x: q.x * inv, y: q.y * inv, z: q.z * inv };
}

function qRotate(q: QuatWXYZ, v: Vec3XYZ): Vec3XYZ {
  const tx = 2 * (q.y * v.z - q.z * v.y);
  const ty = 2 * (q.z * v.x - q.x * v.z);
  const tz = 2 * (q.x * v.y - q.y * v.x);
  return {
    x: v.x + q.w * tx + (q.y * tz - q.z * ty),
    y: v.y + q.w * ty + (q.z * tx - q.x * tz),
    z: v.z + q.w * tz + (q.x * ty - q.y * tx),
  };
}

function qFromAxisAngle(ax: number, ay: number, az: number, angleRad: number): QuatWXYZ {
  const half = 0.5 * angleRad;
  const s = Math.sin(half);
  return { w: Math.cos(half), x: ax * s, y: ay * s, z: az * s };
}

function qShortestArc(from: Vec3XYZ, to: Vec3XYZ): QuatWXYZ {
  const dot = Math.max(-1, Math.min(1, from.x * to.x + from.y * to.y + from.z * to.z));
  if (dot < -0.99999) {
    const ortho: Vec3XYZ = Math.abs(from.x) < 0.9 ? { x: 1, y: 0, z: 0 } : { x: 0, y: 0, z: 1 };
    const cx = from.y * ortho.z - from.z * ortho.y;
    const cy = from.z * ortho.x - from.x * ortho.z;
    const cz = from.x * ortho.y - from.y * ortho.x;
    const n = Math.hypot(cx, cy, cz);
    return n > 1e-9 ? { w: 0, x: cx / n, y: cy / n, z: cz / n } : { w: 0, x: 1, y: 0, z: 0 };
  }
  return qNorm({
    w: 1 + dot,
    x: from.y * to.z - from.z * to.y,
    y: from.z * to.x - from.x * to.z,
    z: from.x * to.y - from.y * to.x,
  });
}

function qSlerp(a: QuatWXYZ, b: QuatWXYZ, t: number): QuatWXYZ {
  let bx = b.x, by = b.y, bz = b.z, bw = b.w;
  let dot = a.w * bw + a.x * bx + a.y * by + a.z * bz;
  if (dot < 0) {
    dot = -dot;
    bx = -bx; by = -by; bz = -bz; bw = -bw;
  }
  if (dot > 0.9995) {
    return qNorm({
      w: a.w + (bw - a.w) * t,
      x: a.x + (bx - a.x) * t,
      y: a.y + (by - a.y) * t,
      z: a.z + (bz - a.z) * t,
    });
  }
  const theta0 = Math.acos(Math.min(1, dot));
  const sin0 = Math.sin(theta0);
  const s0 = Math.sin((1 - t) * theta0) / sin0;
  const s1 = Math.sin(t * theta0) / sin0;
  return qNorm({
    w: a.w * s0 + bw * s1,
    x: a.x * s0 + bx * s1,
    y: a.y * s0 + by * s1,
    z: a.z * s0 + bz * s1,
  });
}

/**
 * Exact group-homomorphic conversion from Right-Handed World Frame (+X Left, +Y Up, +Z Forward)
 * to Babylon.js Left-Handed World Frame (+X Right, +Y Up, +Z Forward):
 *   q_bab = (+qx, -qy, -qz, +qw)
 */
function toBabylonQuat(q: QuatWXYZ): QuatXYZW {
  return { x: q.x, y: -q.y, z: -q.z, w: q.w };
}

class BiquadLowPass3 {
  private b0 = 1;
  private b1 = 0;
  private b2 = 0;
  private a1 = 0;
  private a2 = 0;
  private x1: Vec3XYZ = { x: 0, y: 0, z: 0 };
  private x2: Vec3XYZ = { x: 0, y: 0, z: 0 };
  private y1: Vec3XYZ = { x: 0, y: 0, z: 0 };
  private y2: Vec3XYZ = { x: 0, y: 0, z: 0 };
  private initialized = false;

  constructor(fcHz: number, fsHz: number) {
    const k = Math.tan((Math.PI * fcHz) / fsHz);
    const norm = 1 / (1 + Math.SQRT2 * k + k * k);
    this.b0 = k * k * norm;
    this.b1 = 2 * this.b0;
    this.b2 = this.b0;
    this.a1 = 2 * (k * k - 1) * norm;
    this.a2 = (1 - Math.SQRT2 * k + k * k) * norm;
  }

  reset(v: Vec3XYZ): void {
    this.x1 = { ...v };
    this.x2 = { ...v };
    this.y1 = { ...v };
    this.y2 = { ...v };
    this.initialized = true;
  }

  step(x: Vec3XYZ): Vec3XYZ {
    if (!this.initialized) {
      this.reset(x);
      return { ...x };
    }
    const y: Vec3XYZ = {
      x: this.b0 * x.x + this.b1 * this.x1.x + this.b2 * this.x2.x - this.a1 * this.y1.x - this.a2 * this.y2.x,
      y: this.b0 * x.y + this.b1 * this.x1.y + this.b2 * this.x2.y - this.a1 * this.y1.y - this.a2 * this.y2.y,
      z: this.b0 * x.z + this.b1 * this.x1.z + this.b2 * this.x2.z - this.a1 * this.y1.z - this.a2 * this.y2.z,
    };
    this.x2 = this.x1;
    this.x1 = { ...x };
    this.y2 = this.y1;
    this.y1 = y;
    return y;
  }
}

/**
 * Computes exact lowest-point clearance above the floor (y = 0) for the rotated BalaX
 * geometry (wheels of radius `wheelRadius` at x = +-0.034 m, chassis from y = -0.003..+0.082 m,
 * z = -0.012..+0.012 m) so the 3D robot rests naturally on the floor in ANY 3D orientation.
 */
function computeMinFloorY(qW: QuatWXYZ, wheelRadius: number, halfTrack: number): number {
  let minY = 0;

  // 1. Check both cylindrical wheel rims
  const wheelAxisW = qRotate(qW, { x: 1, y: 0, z: 0 });
  const rimVerticalRadius = wheelRadius * Math.sqrt(Math.max(0, 1 - wheelAxisW.y * wheelAxisW.y));
  for (const sx of [-1, 1]) {
    const hub = qRotate(qW, { x: sx * halfTrack, y: 0, z: 0 });
    minY = Math.min(minY, hub.y - rimVerticalRadius);
  }

  // 2. Check M5StickC Plus + BalaC cradle extremal corners
  const corners: readonly Vec3XYZ[] = [
    { x: -0.013, y: 0.082, z: -0.008 },
    { x: 0.013, y: 0.082, z: -0.008 },
    { x: -0.013, y: 0.082, z: 0.008 },
    { x: 0.013, y: 0.082, z: 0.008 },
    { x: -0.028, y: -0.003, z: -0.012 },
    { x: 0.028, y: -0.003, z: -0.012 },
    { x: -0.028, y: -0.003, z: 0.012 },
    { x: 0.028, y: -0.003, z: 0.012 },
  ];
  for (const c of corners) {
    const p = qRotate(qW, c);
    if (p.y < minY) minY = p.y;
  }

  return Math.max(0.006, -minY);
}

/**
 * Legacy helper kept exported for compatibility / diagnostics.
 */
export function computeImuGravityAttitude(sample: ImuTelemetrySample): {
  targetPitchRad: number;
  targetRollRad: number;
  pitchRateRad: number;
  rollRateRad: number;
  yawRateRad: number;
} {
  const ax = -(sample.accelXG ?? 0);
  const ay = -(sample.accelYG ?? -1);
  const az = sample.accelZG ?? 0;
  const norm = Math.hypot(ax, ay, az);
  const targetPitchRad = norm > 0.25 ? Math.atan2(az, ay) : sample.pitchDeg * DEG_TO_RAD;
  const targetRollRad = norm > 0.25 ? Math.atan2(ax, Math.hypot(ay, az)) : 0;
  return {
    targetPitchRad,
    targetRollRad,
    pitchRateRad: (sample.gyroXDps ?? sample.pitchDps ?? 0) * DEG_TO_RAD,
    rollRateRad: (sample.gyroZDps ?? 0) * DEG_TO_RAD,
    yawRateRad: -(sample.gyroYDps ?? 0) * DEG_TO_RAD,
  };
}

export interface WheeledRobotPhysicsSimulator {
  /** Feed newly arrived 100 Hz telemetry samples (in chronological order; overlapping tails are deduplicated). */
  pushSamples(samples: readonly ImuTelemetrySample[]): void;
  /** Feed a single latest telemetry sample. */
  setLatestSample(sample: ImuTelemetrySample): void;
  /** Zero world yaw heading and recenter 3D linear position (X, Y, Z) & velocity. */
  resetPose(): void;
  /** Reset online residual bias & pose (e.g. after running firmware `imu zero`). */
  resetAll(): void;
  /** Advance the 6-DOF rigid-body kinematics by `dtSeconds` (called every Babylon render frame). */
  step(dtSeconds: number): RobotKinematicState;
  /** Update CG->IMU lever-arm offset (meters, body frame) live without resetting state. */
  setLeverArm(r: readonly [number, number, number]): void;
  /** Read current state without stepping. */
  getState(): RobotKinematicState;
}

export function createWheeledRobotPhysics(
  options: WheeledRobotPhysicsOptions = {}
): WheeledRobotPhysicsSimulator {
  const wheelRadius = options.wheelRadiusM ?? 0.021;
  const trackWidth = options.trackWidthM ?? 0.068;
  const halfTrack = trackWidth * 0.5;
  const maxWheelOmega = options.maxWheelOmegaRadPerSec ?? 14.0;
  const motorTau = options.motorTauSec ?? 0.035;
  const arenaLimit = options.arenaLimitM ?? 1.2;
  const enableLinearMotion = options.enableLinearMotion ?? true;

  // 100 Hz sample queue
  const sampleQueue: ImuTelemetrySample[] = [];
  let currentSample: ImuTelemetrySample | null = null;
  let playbackCarrySec = 0;
  let lastQueuedTick = 0;

  // VQF 6-DOF Quaternion Filter state (Right-Handed World Frame W: +X Left, +Y Up, +Z Forward)
  let seeded = false;
  let qGyr: QuatWXYZ = { w: 1, x: 0, y: 0, z: 0 };
  let qCorr: QuatWXYZ = { w: 1, x: 0, y: 0, z: 0 };
  let qFused: QuatWXYZ = { w: 1, x: 0, y: 0, z: 0 };
  let qRender: QuatWXYZ = { w: 1, x: 0, y: 0, z: 0 };
  let accLp = new BiquadLowPass3(0.6, 100);

  // CG->IMU lever-arm offset in body frame (+X Left, +Y Up, +Z Forward), meters
  let leverArm: Vec3XYZ = {
    x: options.leverArmM?.[0] ?? 0,
    y: options.leverArmM?.[1] ?? 0,
    z: options.leverArmM?.[2] ?? 0,
  };
  const gyroHist: Vec3XYZ[] = [];

  // Windowed-RMS rest detection & online residual bias adaptation
  let accMean: Vec3XYZ = { x: 0, y: 1, z: 0 };
  let gyroMean: Vec3XYZ = { x: 0, y: 0, z: 0 };
  let accVar = 0;
  let gyroVar = 0;
  let onlineGyroBiasRad: Vec3XYZ = { x: 0, y: 0, z: 0 };
  let restTimeSec = 0;
  let resting = false;

  // 3-Axis Linear Motion State (Right-Handed World Frame W, meters & m/s)
  let velW: Vec3XYZ = { x: 0, y: 0, z: 0 };
  let posW: Vec3XYZ = { x: 0, y: wheelRadius, z: 0 };
  let posRenderW: Vec3XYZ = { x: 0, y: wheelRadius, z: 0 };
  let prevLinAccW: Vec3XYZ = { x: 0, y: 0, z: 0 };
  let worldAccRestOffset: Vec3XYZ = { x: 0, y: 0, z: 0 };
  let lowAccTimeSec = 0;
  let lastGyroBodyRad: Vec3XYZ = { x: 0, y: 0, z: 0 };

  const state: RobotKinematicState = {
    quat: { x: 0, y: 0, z: 0, w: 1 },
    pitchRad: 0,
    pitchRateRadPerSec: 0,
    rollRad: 0,
    rollRateRadPerSec: 0,
    yawRad: 0,
    yawRateRadPerSec: 0,
    wheelAngleL: 0,
    wheelAngleR: 0,
    wheelOmegaL: 0,
    wheelOmegaR: 0,
    posX: 0,
    posY: wheelRadius,
    posZ: 0,
    axleHeightY: wheelRadius,
    floorY: wheelRadius,
    vel: { x: 0, y: 0, z: 0 },
    linearAccel: { x: 0, y: 0, z: 0 },
    isResting: false,
  };

  function headingRad(q: QuatWXYZ): number {
    const fwd = qRotate(q, { x: 0, y: 0, z: 1 });
    if (Math.hypot(fwd.x, fwd.z) > 0.2) {
      return Math.atan2(fwd.x, fwd.z);
    }
    const up = qRotate(q, { x: 0, y: 1, z: 0 });
    const sign = fwd.y >= 0 ? -1 : 1;
    return Math.atan2(sign * up.x, sign * up.z);
  }

  function softDeadband(v: number, thresh: number): number {
    if (Math.abs(v) <= thresh) return 0;
    return v > 0 ? v - thresh : v + thresh;
  }

  function processSample100Hz(s: ImuTelemetrySample, dt: number): void {
    // Map M5StickC Plus MPU6886 (+X Right, +Y Down, +Z Forward) -> Right-Handed Body Frame B (+X Left, +Y Up, +Z Forward)
    const has6Axis =
      s.accelXG !== undefined ||
      s.accelYG !== undefined ||
      s.accelZG !== undefined;

    const aB: Vec3XYZ = has6Axis
      ? { x: -(s.accelXG ?? 0), y: -(s.accelYG ?? -1), z: s.accelZG ?? 0 }
      : {
          x: 0,
          y: Math.cos(s.pitchDeg * DEG_TO_RAD),
          z: Math.sin(s.pitchDeg * DEG_TO_RAD),
        };

    const wB: Vec3XYZ = {
      x: -(s.gyroXDps ?? s.pitchDps ?? 0) * DEG_TO_RAD,
      y: -(s.gyroYDps ?? 0) * DEG_TO_RAD,
      z: (s.gyroZDps ?? 0) * DEG_TO_RAD,
    };

    const aNorm = Math.hypot(aB.x, aB.y, aB.z);

    if (!seeded && aNorm > 0.25) {
      const aUnit: Vec3XYZ = { x: aB.x / aNorm, y: aB.y / aNorm, z: aB.z / aNorm };
      qGyr = { w: 1, x: 0, y: 0, z: 0 };
      qCorr = qShortestArc(aUnit, { x: 0, y: 1, z: 0 });
      qFused = { ...qCorr };
      qRender = { ...qCorr };
      accLp = new BiquadLowPass3(0.6, 100);
      accLp.reset(aUnit);
      accMean = { ...aB };
      gyroMean = { ...wB };
      const fy = computeMinFloorY(qFused, wheelRadius, halfTrack);
      posW = { x: 0, y: fy, z: 0 };
      posRenderW = { ...posW };
      velW = { x: 0, y: 0, z: 0 };
      seeded = true;
      return;
    }

    // 1. Windowed-RMS rest detection & online residual thermal gyro bias tracking
    const alphaStat = 1 - Math.exp(-dt / 0.20);
    const alphaVar = 1 - Math.exp(-dt / 0.12);
    accMean.x += (aB.x - accMean.x) * alphaStat;
    accMean.y += (aB.y - accMean.y) * alphaStat;
    accMean.z += (aB.z - accMean.z) * alphaStat;
    gyroMean.x += (wB.x - gyroMean.x) * alphaStat;
    gyroMean.y += (wB.y - gyroMean.y) * alphaStat;
    gyroMean.z += (wB.z - gyroMean.z) * alphaStat;

    const wxCorr = wB.x - onlineGyroBiasRad.x;
    const wyCorr = wB.y - onlineGyroBiasRad.y;
    const wzCorr = wB.z - onlineGyroBiasRad.z;
    const wMag = Math.hypot(wxCorr, wyCorr, wzCorr);
    const dax = aB.x - accMean.x;
    const day = aB.y - accMean.y;
    const daz = aB.z - accMean.z;
    const aDev2 = dax * dax + day * day + daz * daz;
    const aDev = Math.sqrt(aDev2);
    const dwx = wB.x - gyroMean.x;
    const dwy = wB.y - gyroMean.y;
    const dwz = wB.z - gyroMean.z;
    const wDev2 = dwx * dwx + dwy * dwy + dwz * dwz;

    accVar += (aDev2 - accVar) * alphaVar;
    gyroVar += (wDev2 - gyroVar) * alphaVar;
    const aRms = Math.sqrt(Math.max(0, accVar));
    const wRms = Math.sqrt(Math.max(0, gyroVar));

    if (wMag > 10.0 * DEG_TO_RAD || aDev > 0.085) {
      restTimeSec = 0;
    } else if (wMag < 3.5 * DEG_TO_RAD && wRms < 2.6 * DEG_TO_RAD && aRms < 0.034) {
      restTimeSec = Math.min(restTimeSec + dt, 10);
    } else {
      restTimeSec = Math.max(0, restTimeSec - 3 * dt);
    }
    resting = restTimeSec >= 0.30;

    if (resting) {
      const alphaB = 1 - Math.exp(-dt / 4.0);
      const maxBias = 3.0 * DEG_TO_RAD;
      onlineGyroBiasRad.x = Math.max(-maxBias, Math.min(maxBias, onlineGyroBiasRad.x + (wB.x - onlineGyroBiasRad.x) * alphaB));
      onlineGyroBiasRad.y = Math.max(-maxBias, Math.min(maxBias, onlineGyroBiasRad.y + (wB.y - onlineGyroBiasRad.y) * alphaB));
      onlineGyroBiasRad.z = Math.max(-maxBias, Math.min(maxBias, onlineGyroBiasRad.z + (wB.z - onlineGyroBiasRad.z) * alphaB));
    }

    // 2. Exact SO(3) exponential-map gyroscope integration
    const w: Vec3XYZ = {
      x: wB.x - onlineGyroBiasRad.x,
      y: wB.y - onlineGyroBiasRad.y,
      z: wB.z - onlineGyroBiasRad.z,
    };
    const wNorm = Math.hypot(w.x, w.y, w.z);
    const activeW = !resting || wNorm > 0.35 * DEG_TO_RAD ? w : { x: 0, y: 0, z: 0 };
    lastGyroBodyRad = activeW;
    const activeWNorm = Math.hypot(activeW.x, activeW.y, activeW.z);
    if (activeWNorm > 1e-7) {
      const dq = qFromAxisAngle(
        activeW.x / activeWNorm,
        activeW.y / activeWNorm,
        activeW.z / activeWNorm,
        activeWNorm * dt
      );
      qGyr = qNorm(qMul(qGyr, dq));
    }

    // 2b. IMU-to-CG lever-arm acceleration compensation:
    //     a_cg = a_imu - (alpha x r + w x (w x r)) / g
    gyroHist.push({ ...activeW });
    if (gyroHist.length > 5) gyroHist.shift();
    let aCgB: Vec3XYZ = aB;
    const hasLever = Math.hypot(leverArm.x, leverArm.y, leverArm.z) > 1e-4;
    if (hasLever && gyroHist.length >= 2) {
      let ax = 0, ay = 0, az = 0;
      if (gyroHist.length === 5) {
        const w0 = gyroHist[0]!, w1 = gyroHist[1]!, w3 = gyroHist[3]!, w4 = gyroHist[4]!;
        const inv10dt = 1 / (10 * dt);
        ax = (-2 * w0.x - w1.x + w3.x + 2 * w4.x) * inv10dt;
        ay = (-2 * w0.y - w1.y + w3.y + 2 * w4.y) * inv10dt;
        az = (-2 * w0.z - w1.z + w3.z + 2 * w4.z) * inv10dt;
      } else {
        const prev = gyroHist[gyroHist.length - 2]!;
        ax = (activeW.x - prev.x) / dt;
        ay = (activeW.y - prev.y) / dt;
        az = (activeW.z - prev.z) / dt;
      }
      const rx = leverArm.x, ry = leverArm.y, rz = leverArm.z;
      const tangX = ay * rz - az * ry;
      const tangY = az * rx - ax * rz;
      const tangZ = ax * ry - ay * rx;
      const wDotR = activeW.x * rx + activeW.y * ry + activeW.z * rz;
      const w2 = activeW.x * activeW.x + activeW.y * activeW.y + activeW.z * activeW.z;
      const centX = wDotR * activeW.x - w2 * rx;
      const centY = wDotR * activeW.y - w2 * ry;
      const centZ = wDotR * activeW.z - w2 * rz;
      aCgB = {
        x: aB.x - (tangX + centX) / G_MS2,
        y: aB.y - (tangY + centY) / G_MS2,
        z: aB.z - (tangZ + centZ) / G_MS2,
      };
    }

    // 3. VQF Inertial-frame low-pass accelerometer gravity alignment (yaw-preserving!)
    if (aNorm > 0.2) {
      const aInert = qRotate(qGyr, aCgB);
      const aLp = accLp.step(aInert);
      const lpNorm = Math.hypot(aLp.x, aLp.y, aLp.z);
      if (lpNorm > 0.2) {
        const gErr = aNorm - 1.0;
        const trust = Math.exp(-(gErr * gErr) / (2 * 0.14 * 0.14));
        const gain = (1 - Math.exp(-dt / (resting ? 0.30 : 1.0))) * trust;

        const gEstW = qRotate(qCorr, { x: aLp.x / lpNorm, y: aLp.y / lpNorm, z: aLp.z / lpNorm });
        const errX = -gEstW.z;
        const errZ = gEstW.x;
        const errSin = Math.hypot(errX, errZ);
        if (errSin > 1e-6 && gain > 1e-6) {
          const errAngle = Math.atan2(errSin, Math.max(-1, Math.min(1, gEstW.y)));
          const dqTilt = qFromAxisAngle(errX / errSin, 0, errZ / errSin, errAngle * gain);
          qCorr = qNorm(qMul(dqTilt, qCorr));
        }
      }
    }

    qFused = qNorm(qMul(qCorr, qGyr));

    // 4. Wheel motor dynamics
    const targetOmegaL = (s.motorL / 127) * maxWheelOmega;
    const targetOmegaR = (s.motorR / 127) * maxWheelOmega;
    const alphaMotor = 1 - Math.exp(-dt / motorTau);
    state.wheelOmegaL += (targetOmegaL - state.wheelOmegaL) * alphaMotor;
    state.wheelOmegaR += (targetOmegaR - state.wheelOmegaR) * alphaMotor;
    state.wheelAngleL += state.wheelOmegaL * dt;
    state.wheelAngleR += state.wheelOmegaR * dt;

    // 5. 3-Axis World Linear Motion (Gravity-compensated IMU + Rotation-Aware Deadband + ZUPT + Floor)
    const floorY = computeMinFloorY(qFused, wheelRadius, halfTrack);
    state.floorY = floorY;

    const aWorldG = qRotate(qFused, aCgB);
    const rawLinAccW: Vec3XYZ = {
      x: aWorldG.x * G_MS2,
      y: (aWorldG.y - 1.0) * G_MS2,
      z: aWorldG.z * G_MS2,
    };

    // Learn tiny residual resting linear acceleration offset so stationary drift is zero
    if (resting) {
      const alphaOff = 1 - Math.exp(-dt / 0.35);
      worldAccRestOffset.x += (rawLinAccW.x - worldAccRestOffset.x) * alphaOff;
      worldAccRestOffset.y += (rawLinAccW.y - worldAccRestOffset.y) * alphaOff;
      worldAccRestOffset.z += (rawLinAccW.z - worldAccRestOffset.z) * alphaOff;
    } else {
      const decayOff = Math.exp(-dt / 4.0);
      worldAccRestOffset.x *= decayOff;
      worldAccRestOffset.y *= decayOff;
      worldAccRestOffset.z *= decayOff;
    }

    // Rotation-aware deadband suppresses lever-arm/centripetal spikes when spinning in place
    // while keeping tight sensitivity for clean linear hand translations.
    const linDeadband = hasLever ? 0.14 + 0.04 * activeWNorm : 0.16 + 0.09 * activeWNorm;
    const linAccW: Vec3XYZ = {
      x: softDeadband(rawLinAccW.x - worldAccRestOffset.x, linDeadband),
      y: softDeadband(rawLinAccW.y - worldAccRestOffset.y, linDeadband),
      z: softDeadband(rawLinAccW.z - worldAccRestOffset.z, linDeadband),
    };
    const linAccMag = Math.hypot(linAccW.x, linAccW.y, linAccW.z);

    if (linAccMag < 0.12) {
      lowAccTimeSec = Math.min(lowAccTimeSec + dt, 5);
    } else {
      lowAccTimeSec = 0;
    }

    if (enableLinearMotion) {
      // Trapezoidal (midpoint) velocity integration
      velW.x += 0.5 * (prevLinAccW.x + linAccW.x) * dt;
      velW.y += 0.5 * (prevLinAccW.y + linAccW.y) * dt;
      velW.z += 0.5 * (prevLinAccW.z + linAccW.z) * dt;

      // ZUPT (Zero-Velocity Update): stop cleanly when hand or robot comes to rest
      const zuptActive = resting || lowAccTimeSec >= 0.14;
      const dampTau = zuptActive ? 0.05 : 0.70;
      const damp = Math.exp(-dt / dampTau);
      velW.x *= damp;
      velW.y *= damp;
      velW.z *= damp;

      if (zuptActive && Math.hypot(velW.x, velW.y, velW.z) < 0.004) {
        velW.x = 0;
        velW.y = 0;
        velW.z = 0;
      }

      // Fuse differential wheel odometry when motors are actively driving on the floor
      const bodyUpW = qRotate(qFused, { x: 0, y: 1, z: 0 });
      const uprightContact = Math.max(0, bodyUpW.y);
      const motorsActive = Math.abs(s.motorL) + Math.abs(s.motorR) > 2;
      if (motorsActive && uprightContact > 0.4 && posW.y <= floorY + 0.015) {
        const yaw = headingRad(qFused);
        const vWheel = (state.wheelOmegaL + state.wheelOmegaR) * 0.5 * wheelRadius * 0.65 * uprightContact;
        const wVx = Math.sin(yaw) * vWheel;
        const wVz = Math.cos(yaw) * vWheel;
        const blend = 1 - Math.exp(-dt / 0.06);
        velW.x += (wVx - velW.x) * blend;
        velW.z += (wVz - velW.z) * blend;
      }

      // Integrate 3D position
      posW.x = Math.max(-arenaLimit, Math.min(arenaLimit, posW.x + velW.x * dt));
      posW.y = Math.min(arenaLimit, posW.y + velW.y * dt);
      posW.z = Math.max(-arenaLimit, Math.min(arenaLimit, posW.z + velW.z * dt));

      // Ground contact constraint (robot rests on floor plane y = 0 without clipping)
      if (posW.y < floorY) {
        posW.y = floorY;
        if (velW.y < 0) velW.y = 0;
      }
    } else {
      posW.y = floorY;
    }

    prevLinAccW = linAccW;

    // Publish Babylon Left-Handed linear acceleration (+X Right = -W.x)
    state.linearAccel = { x: -linAccW.x, y: linAccW.y, z: linAccW.z };
    state.vel = { x: -velW.x, y: velW.y, z: velW.z };
    state.isResting = resting;
  }

  function updatePublishedState(q: QuatWXYZ, pW: Vec3XYZ): void {
    state.quat = toBabylonQuat(q);

    const qConj: QuatWXYZ = { w: q.w, x: -q.x, y: -q.y, z: -q.z };
    const upB = qRotate(qConj, { x: 0, y: 1, z: 0 });
    state.pitchRad = Math.atan2(upB.z, upB.y);
    state.rollRad = Math.atan2(upB.x, Math.hypot(upB.y, upB.z));
    state.yawRad = headingRad(q);

    state.pitchRateRadPerSec = -lastGyroBodyRad.x;
    state.rollRateRadPerSec = lastGyroBodyRad.z;
    state.yawRateRadPerSec = lastGyroBodyRad.y;

    // Map RH World pW (+X Left) -> Babylon (+X Right = -pW.x)
    state.posX = -pW.x;
    state.posY = pW.y;
    state.posZ = pW.z;
    state.axleHeightY = pW.y;
  }

  function pushSamples(samples: readonly ImuTelemetrySample[]): void {
    if (samples.length === 0) return;

    // Only treat as a robot reboot if the NEWEST tick in the batch jumped backwards significantly.
    // Overlapping slices (e.g. history.slice(-24)) start with older ticks followed by newer ticks
    // and must simply skip s.tick <= lastQueuedTick without resetting!
    const newestTick = samples[samples.length - 1]!.tick;
    if (lastQueuedTick > 0 && newestTick + 25 < lastQueuedTick) {
      sampleQueue.length = 0;
      lastQueuedTick = 0;
      seeded = false;
    }

    for (const s of samples) {
      if (s.tick > lastQueuedTick) {
        sampleQueue.push(s);
        lastQueuedTick = s.tick;
      }
    }
    if (sampleQueue.length > 36) {
      sampleQueue.splice(0, sampleQueue.length - 20);
    }
    if (!currentSample && sampleQueue.length > 0) {
      currentSample = sampleQueue.shift()!;
      processSample100Hz(currentSample, 0.01);
      updatePublishedState(qFused, posW);
    }
  }

  function setLatestSample(sample: ImuTelemetrySample): void {
    if (sample.tick > lastQueuedTick || (lastQueuedTick > 0 && sample.tick + 25 < lastQueuedTick)) {
      pushSamples([sample]);
    } else if (!currentSample) {
      currentSample = sample;
      processSample100Hz(sample, 0.01);
      updatePublishedState(qFused, posW);
    }
  }

  function resetPose(): void {
    if (seeded) {
      const yaw = headingRad(qFused);
      const dq = qFromAxisAngle(0, 1, 0, -yaw);
      qCorr = qNorm(qMul(dq, qCorr));
      qFused = qNorm(qMul(qCorr, qGyr));
      qRender = { ...qFused };
    }
    const fy = computeMinFloorY(qFused, wheelRadius, halfTrack);
    posW = { x: 0, y: fy, z: 0 };
    posRenderW = { ...posW };
    velW = { x: 0, y: 0, z: 0 };
    prevLinAccW = { x: 0, y: 0, z: 0 };
    state.floorY = fy;
    updatePublishedState(qFused, posW);
  }

  function resetAll(): void {
    onlineGyroBiasRad = { x: 0, y: 0, z: 0 };
    worldAccRestOffset = { x: 0, y: 0, z: 0 };
    accVar = 0;
    gyroVar = 0;
    gyroHist.length = 0;
    restTimeSec = 0;
    resetPose();
  }

  function setLeverArm(r: readonly [number, number, number]): void {
    leverArm = { x: r[0] ?? 0, y: r[1] ?? 0, z: r[2] ?? 0 };
  }

  function step(dtSeconds: number): RobotKinematicState {
    const dt = Math.max(0.001, Math.min(0.05, Number.isFinite(dtSeconds) ? dtSeconds : 1 / 60));

    // Process queued 100 Hz samples at true 100 Hz (with gentle catch-up if queue builds)
    let consumed = 0;
    if (sampleQueue.length > 0) {
      const catchUpRate = sampleQueue.length > 14 ? 1.35 : sampleQueue.length > 6 ? 1.1 : 1.0;
      playbackCarrySec += dt * catchUpRate;
      const samplePeriod = 0.01;
      while (playbackCarrySec >= samplePeriod && sampleQueue.length > 0) {
        playbackCarrySec -= samplePeriod;
        currentSample = sampleQueue.shift()!;
        processSample100Hz(currentSample, samplePeriod);
        consumed++;
      }
    } else {
      playbackCarrySec = 0;
    }

    if (!seeded) {
      return state;
    }

    // Silky 60-120 fps render interpolation for both quaternion and 3D position
    const smoothQ = 1 - Math.exp(-dt * 38);
    if (consumed === 0 && !resting) {
      const wNorm = Math.hypot(lastGyroBodyRad.x, lastGyroBodyRad.y, lastGyroBodyRad.z);
      if (wNorm > 1e-4) {
        const dqPred = qFromAxisAngle(
          lastGyroBodyRad.x / wNorm,
          lastGyroBodyRad.y / wNorm,
          lastGyroBodyRad.z / wNorm,
          wNorm * dt * 0.45
        );
        qRender = qSlerp(qNorm(qMul(qRender, dqPred)), qFused, smoothQ);
      } else {
        qRender = qSlerp(qRender, qFused, smoothQ);
      }
    } else {
      qRender = qSlerp(qRender, qFused, smoothQ);
    }

    const smoothP = 1 - Math.exp(-dt * 32);
    posRenderW.x += (posW.x - posRenderW.x) * smoothP;
    posRenderW.y += (posW.y - posRenderW.y) * smoothP;
    posRenderW.z += (posW.z - posRenderW.z) * smoothP;

    updatePublishedState(qRender, posRenderW);
    return state;
  }

  return {
    pushSamples,
    setLatestSample,
    resetPose,
    resetAll,
    setLeverArm,
    step,
    getState: () => state,
  };
}
