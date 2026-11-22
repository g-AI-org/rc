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
