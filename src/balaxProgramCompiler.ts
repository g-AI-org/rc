// In-browser compiler for BalaX sandboxed Mini-Programs (BXVM / BXP1 bytecode).
// Compiles a tiny readable script syntax into compact BXP1 bytecode that runs on Core 0
// with zero flash wear via BalaX's Tier-0 DRAM Applet Cache.

export interface CompileResult {
  ok: boolean;
  bytes?: Uint8Array;
  codeBytes?: number;
  stringBytes?: number;
  error?: string;
  line?: number;
}

export interface ProgramTemplate {
  id: string;
  name: string;
  description: string;
  source: string;
}

const OP = {
  HALT: 0x00,
  PUSH_I32: 0x01,
  POP: 0x02,
  DUP: 0x03,
  ADD: 0x10,
  SUB: 0x11,
  MUL: 0x12,
  DIV: 0x13,
  MOD: 0x14,
  NEG: 0x15,
  CMP_EQ: 0x20,
  CMP_NE: 0x21,
  CMP_LT: 0x22,
  CMP_LE: 0x23,
  CMP_GT: 0x24,
  CMP_GE: 0x25,
  JMP: 0x30,
  JZ: 0x31,
  JNZ: 0x32,
  LOAD_VAR: 0x40,
  STORE_VAR: 0x41,
  SYSCALL: 0x50,
} as const;

const SYSCALL: Record<string, { id: number; args: number; strArgIdx?: number }> = {
  beep:        { id: 0x01, args: 2 },
  led:         { id: 0x02, args: 1 },
  sleep:       { id: 0x03, args: 1 },
  sleep_ms:    { id: 0x03, args: 1 },
  set_stick:   { id: 0x04, args: 2 },
  arm:         { id: 0x05, args: 0 },
  disarm:      { id: 0x06, args: 0 },
  takeoff:     { id: 0x07, args: 0 },
  state:       { id: 0x08, args: 0 },
  pitch_cdeg:  { id: 0x09, args: 0 },
  roll_cdeg:   { id: 0x0a, args: 0 },
  yaw_cdeg:    { id: 0x0b, args: 0 },
  battery:     { id: 0x0c, args: 0 },
  pmic_dc:     { id: 0x0d, args: 0 },
  uptime_ms:   { id: 0x0e, args: 0 },
  lcd_clear:   { id: 0x0f, args: 1 },
  lcd_text:    { id: 0x10, args: 5, strArgIdx: 2 },
  lcd_bar:     { id: 0x11, args: 3 },
  log:         { id: 0x12, args: 2 },
};

const CONSTANTS: Record<string, number> = {
  COL_BG: 0x0000,
  COL_TEXT: 0xffff,
  COL_WHITE: 0xffff,
  COL_MUTED: 0x8410,
  COL_BLUE: 0x5d3f,
  COL_GREEN: 0x3dc9,
  COL_AMBER: 0xd4c4,
  COL_RED: 0xf289,
  TRUE: 1,
  FALSE: 0,
  true: 1,
  false: 0,
};

type TokKind = "num" | "str" | "id" | "op" | "eof";
interface Tok {
  kind: TokKind;
  val: string;
  line: number;
}

function tokenize(src: string): Tok[] {
  const toks: Tok[] = [];
  let i = 0;
  let line = 1;
  while (i < src.length) {
    const c = src[i];
    if (c === "\n") { line++; i++; continue; }
    if (/\s/.test(c)) { i++; continue; }
    if (c === "/" && src[i + 1] === "/") {
      while (i < src.length && src[i] !== "\n") i++;
      continue;
    }
    if (c === "#") {
      while (i < src.length && src[i] !== "\n") i++;
      continue;
    }
    if (c === '"') {
      let s = "";
      i++;
      while (i < src.length && src[i] !== '"') {
        if (src[i] === "\\" && i + 1 < src.length) {
          i++;
          s += src[i] === "n" ? "\n" : src[i];
        } else {
          s += src[i];
        }
        i++;
      }
      if (i >= src.length) throw { message: "Unterminated string literal", line };
      i++;
      toks.push({ kind: "str", val: s, line });
      continue;
    }
    if (/[0-9]/.test(c)) {
      let n = "";
      if (c === "0" && (src[i + 1] === "x" || src[i + 1] === "X")) {
        n = "0x";
        i += 2;
        while (i < src.length && /[0-9a-fA-F]/.test(src[i])) n += src[i++];
      } else {
        while (i < src.length && /[0-9]/.test(src[i])) n += src[i++];
      }
      toks.push({ kind: "num", val: n, line });
      continue;
    }
    if (/[a-zA-Z_]/.test(c)) {
      let id = "";
      while (i < src.length && /[a-zA-Z0-9_]/.test(src[i])) id += src[i++];
      toks.push({ kind: "id", val: id, line });
      continue;
    }
    const two = src.slice(i, i + 2);
    if (["==", "!=", "<=", ">="].includes(two)) {
      toks.push({ kind: "op", val: two, line });
      i += 2;
      continue;
    }
    if ("+-*/%()={},;<>".includes(c)) {
      toks.push({ kind: "op", val: c, line });
      i++;
      continue;
    }
    throw { message: `Unexpected character '${c}'`, line };
  }
  toks.push({ kind: "eof", val: "", line });
  return toks;
}

export function compileBalaXProgram(source: string): CompileResult {
  try {
    const toks = tokenize(source);
    let pos = 0;
    const code: number[] = [];
    const strings: string[] = [];
    const vars = new Map<string, number>();

    const peek = (off = 0) => toks[Math.min(pos + off, toks.length - 1)];
    const eat = (expected?: string): Tok => {
      const t = peek();
      if (expected && t.val !== expected) {
        throw { message: `Expected '${expected}' but got '${t.val || "EOF"}'`, line: t.line };
      }
      pos++;
      return t;
    };
    const emit = (b: number) => code.push(b & 0xff);
    const emitU16 = (v: number) => { emit(v & 0xff); emit((v >> 8) & 0xff); };
    const emitI32 = (v: number) => {
      const u = v >>> 0;
      emit(u & 0xff); emit((u >> 8) & 0xff); emit((u >> 16) & 0xff); emit((u >> 24) & 0xff);
    };
    const patchU16 = (at: number, v: number) => {
      code[at] = v & 0xff;
      code[at + 1] = (v >> 8) & 0xff;
    };
    const varIndex = (name: string, line: number): number => {
      if (vars.has(name)) return vars.get(name)!;
      if (vars.size >= 32) throw { message: "Too many variables (max 32)", line };
      const idx = vars.size;
      vars.set(name, idx);
      return idx;
    };
    const internStr = (s: string): number => {
      const idx = strings.indexOf(s);
      if (idx >= 0) return idx;
      strings.push(s);
      return strings.length - 1;
    };

    const parsePrimary = () => {
      const t = peek();
      if (t.kind === "num") {
        eat();
        emit(OP.PUSH_I32);
        emitI32(t.val.startsWith("0x") ? parseInt(t.val, 16) : parseInt(t.val, 10));
        return;
      }
      if (t.kind === "op" && t.val === "-") {
        eat();
        parsePrimary();
        emit(OP.NEG);
        return;
      }
      if (t.kind === "op" && t.val === "(") {
        eat("(");
        parseExpr();
        eat(")");
        return;
      }
      if (t.kind === "id") {
        if (peek(1).val === "(") {
          parseCall();
          return;
        }
        eat();
        if (Object.prototype.hasOwnProperty.call(CONSTANTS, t.val)) {
          emit(OP.PUSH_I32);
          emitI32(CONSTANTS[t.val]);
          return;
        }
        if (!vars.has(t.val)) {
          throw { message: `Undefined variable '${t.val}'`, line: t.line };
        }
        emit(OP.LOAD_VAR);
        emit(vars.get(t.val)!);
        return;
      }
      throw { message: `Unexpected token '${t.val || "EOF"}' in expression`, line: t.line };
    };

    const parseCall = () => {
      const fn = eat();
      const sc = SYSCALL[fn.val];
      if (!sc) throw { message: `Unknown function '${fn.val}'`, line: fn.line };
      eat("(");
      let argc = 0;
      if (peek().val !== ")") {
        while (true) {
          if (sc.strArgIdx === argc && peek().kind === "str") {
            const st = eat();
            emit(OP.PUSH_I32);
            emitI32(internStr(st.val));
          } else {
            parseExpr();
          }
          argc++;
          if (peek().val === ",") eat(",");
          else break;
        }
      }
      eat(")");
      if (argc !== sc.args) {
        throw { message: `${fn.val}() expects ${sc.args} args, got ${argc}`, line: fn.line };
      }
      emit(OP.SYSCALL);
      emit(sc.id);
    };

    const parseMul = () => {
      parsePrimary();
      while (["*", "/", "%"].includes(peek().val)) {
        const op = eat().val;
        parsePrimary();
        emit(op === "*" ? OP.MUL : op === "/" ? OP.DIV : OP.MOD);
      }
    };

    const parseAdd = () => {
      parseMul();
      while (["+", "-"].includes(peek().val)) {
        const op = eat().val;
        parseMul();
        emit(op === "+" ? OP.ADD : OP.SUB);
      }
    };

    const parseExpr = () => {
      parseAdd();
      while (["==", "!=", "<", "<=", ">", ">="].includes(peek().val)) {
        const op = eat().val;
        parseAdd();
        const map: Record<string, number> = {
          "==": OP.CMP_EQ, "!=": OP.CMP_NE, "<": OP.CMP_LT,
          "<=": OP.CMP_LE, ">": OP.CMP_GT, ">=": OP.CMP_GE,
        };
        emit(map[op]);
      }
    };

    const parseBlock = () => {
      eat("{");
      while (peek().val !== "}" && peek().kind !== "eof") {
        parseStmt();
      }
      eat("}");
    };

    const parseStmt = () => {
      const t = peek();
      if (t.val === ";") { eat(); return; }
      if (t.kind === "id" && (t.val === "let" || t.val === "var")) {
        eat();
        const name = eat();
        if (name.kind !== "id") throw { message: "Expected variable name", line: name.line };
        eat("=");
        parseExpr();
        emit(OP.STORE_VAR);
        emit(varIndex(name.val, name.line));
        if (peek().val === ";") eat(";");
        return;
      }
      if (t.kind === "id" && t.val === "if") {
        eat();
        eat("(");
        parseExpr();
        eat(")");
        emit(OP.JZ);
        const jzPatch = code.length;
        emitU16(0);
        parseBlock();
        if (peek().kind === "id" && peek().val === "else") {
          eat();
          emit(OP.JMP);
          const jmpPatch = code.length;
          emitU16(0);
          patchU16(jzPatch, code.length);
          parseBlock();
          patchU16(jmpPatch, code.length);
        } else {
          patchU16(jzPatch, code.length);
        }
        return;
      }
      if (t.kind === "id" && t.val === "while") {
        eat();
        const loopStart = code.length;
        eat("(");
        parseExpr();
        eat(")");
        emit(OP.JZ);
        const exitPatch = code.length;
        emitU16(0);
        parseBlock();
        emit(OP.JMP);
        emitU16(loopStart);
        patchU16(exitPatch, code.length);
        return;
      }
      if (t.kind === "id" && peek(1).val === "=") {
        const name = eat();
        eat("=");
        parseExpr();
        emit(OP.STORE_VAR);
        emit(varIndex(name.val, name.line));
        if (peek().val === ";") eat(";");
        return;
      }
      // Expression statement (typically a syscall)
      parseExpr();
      emit(OP.POP);
      if (peek().val === ";") eat(";");
    };

    while (peek().kind !== "eof") {
      parseStmt();
    }
    emit(OP.HALT);

    const enc = new TextEncoder();
    const strChunks: Uint8Array[] = [];
    let strTotal = 0;
    for (const s of strings) {
      const b = enc.encode(s);
      strChunks.push(b);
      strTotal += b.byteLength + 1;
    }

    const totalLen = 10 + code.length + strTotal;
    const out = new Uint8Array(totalLen);
    out[0] = 0x42; // 'B'
    out[1] = 0x58; // 'X'
    out[2] = 0x50; // 'P'
    out[3] = 0x31; // '1'
    out[4] = 0;
    out[5] = 0;
    out[6] = code.length & 0xff;
    out[7] = (code.length >> 8) & 0xff;
    out[8] = strTotal & 0xff;
    out[9] = (strTotal >> 8) & 0xff;
    out.set(code, 10);
    let sp = 10 + code.length;
    for (const chunk of strChunks) {
      out.set(chunk, sp);
      sp += chunk.byteLength;
      out[sp++] = 0;
    }
    return { ok: true, bytes: out, codeBytes: code.length, stringBytes: strTotal };
  } catch (e: unknown) {
    const err = e as { message?: string; line?: number };
    return { ok: false, error: err.message ?? String(e), line: err.line };
  }
}

export const BALAX_PROGRAM_TEMPLATES: ProgramTemplate[] = [
  {
    id: "beep-bar",
    name: "Beep & Loading Bar",
    description: "Draws a Micro-BIOS progress bar on the LCD, beeps a rising scale, and logs battery %.",
    source: `// Runs in Core 0 BXVM with zero flash wear!
lcd_clear(COL_BG)
lcd_text(8, 16, "BXVM DEMO", COL_BLUE, 2)
lcd_text(8, 42, "ZERO FLASH WEAR", COL_TEXT, 1)

let pct = 0
while (pct <= 100) {
  lcd_bar(72, pct, COL_GREEN)
  led(pct % 20 == 0)
  beep(600 + pct * 8, 35)
  log(1, pct)
  sleep(120)
  pct = pct + 10
}
log(2, battery())
log(3, pmic_dc())
led(0)
lcd_text(8, 104, "DONE!", COL_GREEN, 2)
sleep(600)
`,
  },
  {
    id: "sensor-inspector",
    name: "Thermal & IMU Inspector",
    description: "Samples pitch, battery %, and PMIC temp 5 times and logs them back to the browser.",
    source: `// Live sensor inspection applet
lcd_clear(COL_BG)
lcd_text(8, 14, "SENSOR CHECK", COL_AMBER, 1)

let i = 0
while (i < 5) {
  let b = battery()
  let t = pmic_dc()
  let p = pitch_cdeg()
  log(10 + i, p)
  log(20 + i, b)
  log(30 + i, t)
  lcd_bar(48, b, COL_BLUE)
  beep(900 + i * 100, 40)
  sleep(250)
  i = i + 1
}
`,
  },
  {
    id: "melody-led",
    name: "LED & Chime Show",
    description: "Plays a 4-note arpeggio while blinking the red M5StickC Plus LED.",
    source: `// Short LED + buzzer arpeggio
let i = 0
while (i < 4) {
  led(1)
  beep(523 + i * 130, 80)
  sleep(110)
  led(0)
  sleep(60)
  i = i + 1
}
log(99, uptime_ms())
`,
  },
  {
    id: "pitch-meter",
    name: "Live Pitch Bar (5s)",
    description: "Displays live tilt angle on the Micro-BIOS progress bar for 5 seconds.",
    source: `// Tilt the robot to watch the Micro-BIOS bar move!
lcd_clear(COL_BG)
lcd_text(8, 16, "PITCH METER", COL_GREEN, 1)

let step = 0
while (step < 50) {
  let p = pitch_cdeg() / 100
  let pct = 50 + p
  if (pct < 0) { pct = 0 }
  if (pct > 100) { pct = 100 }
  lcd_bar(60, pct, COL_BLUE)
  if (step % 10 == 0) {
    log(5, p)
  }
  sleep(100)
  step = step + 1
}
`,
  },
];
