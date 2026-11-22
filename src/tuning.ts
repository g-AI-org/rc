/**
 * Schema-driven runtime parameter / PID gain descriptor.
 * Compatible with esp32-bot's `settings` component (`get`, `set <name> <val>`, `save`, `defaults`).
 */
export interface TuningParameter {
  name: string;
  value: number;
  min: number;
  max: number;
  help: string;
}

const SETTING_LINE_RE = /^(\S+)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\.\.(-?\d+(?:\.\d+)?)\s+(.*)$/gm;

export function parseSettingsTable(consoleText: string): TuningParameter[] {
  const list: TuningParameter[] = [];
  let m: RegExpExecArray | null;
  SETTING_LINE_RE.lastIndex = 0;
  while ((m = SETTING_LINE_RE.exec(consoleText)) !== null) {
    list.push({
      name: m[1],
      value: Number(m[2]),
      min: Number(m[3]),
      max: Number(m[4]),
      help: m[5],
    });
  }
  return list;
}

export interface GainParam {
  name: string;
  value: number;
  min: number;
  max: number;
}

export function createDefaultBalaXGains(): GainParam[] {
  return [
    { name: "k_angle", value: 65, min: 0, max: 300 },
    { name: "k_rate", value: 2, min: 0, max: 50 },
    { name: "k_angle_i", value: 2400, min: 0, max: 10000 },
    { name: "k_rate_i", value: 10, min: 0, max: 200 },
    { name: "k_yaw", value: 4, min: 0, max: 50 },
    { name: "k_distance", value: 300, min: 0, max: 2000 },
    { name: "k_speed", value: 9, min: 0, max: 100 },
    { name: "motor_gain", value: 0.5, min: 0, max: 2 },
    { name: "deadband", value: 5, min: 0, max: 40 },
    { name: "punch", value: 5, min: 0, max: 40 },
    { name: "trim", value: 0, min: -30, max: 30 },
  ];
}
