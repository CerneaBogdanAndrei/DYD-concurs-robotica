// Compară implementarea TS a protocolului cu Arm_Lib.py (biblioteca Yahboom).
//
//   npm run compara:armlib -- codare [--armlib DIR] [--python python3]
//       Încarcă Arm_Lib cu un `smbus` falsificat care înregistrează fiecare apel I2C,
//       apoi compară octet cu octet scrierile (6 servo, 1 servo, cuplu, buzzer) și
//       decodarea citirilor. Nu atinge brațul; merge și pe Windows dacă aveți
//       Arm_Lib.py (DIR = folderul care îl conține).
//
//   npm run compara:armlib -- citire [--python python3]
//       Doar pe Pi: citește cele 6 servo-uri prin TS (i2c-bus), apoi prin Arm_Lib,
//       și compară. Brațul nu se mișcă.
//
// Pe Pi: `pip3 show -f Arm_Lib` arată unde e instalat Arm_Lib.py.

import { spawnSync } from "node:child_process";
import { parseArgs } from "node:util";
import { openI2cArm } from "../src/arm/i2c.js";
import {
  type I2cOp,
  type ServoId,
  BOARD_ADDRESS,
  REG,
  SERVO_IDS,
  decodeRead,
  encodeBuzzer,
  encodeTorque,
  encodeWrite6,
  encodeWriteServo,
  servoRange,
  swapWord,
} from "../src/arm/protocol.js";

const { values, positionals } = parseArgs({
  allowPositionals: true,
  options: {
    armlib: { type: "string", default: "" },
    python: { type: "string", default: process.platform === "win32" ? "python" : "python3" },
  },
});

/** Apeluri SMBus în forma înregistrată de partea Python. */
type Call = [string, ...unknown[]];

interface Case {
  name: string;
  method: string;
  args: unknown[];
  /** cuvântul brut pe care îl întoarce read_word_data (doar pentru citiri) */
  word?: number;
  expectCalls: Call[];
  expectReturn?: number | null;
}

const PY = String.raw`
import sys, json, types, io, contextlib
mode, armdir = sys.argv[1], sys.argv[2]
if armdir:
    sys.path.insert(0, armdir)
calls, state = [], {"word": 0}
if mode == "codare":
    class SMBus:
        def __init__(self, *a, **k): pass
        def write_i2c_block_data(self, a, r, d): calls.append(["block", a, r, [int(x) for x in d]])
        def write_byte_data(self, a, r, v): calls.append(["byte", a, r, int(v)])
        def write_byte(self, a, v): calls.append(["wbyte", a, int(v)])
        def read_word_data(self, a, r): calls.append(["rword", a, r]); return state["word"]
        def read_byte_data(self, a, r): calls.append(["rbyte", a, r]); return 0
        def read_i2c_block_data(self, a, r, n): calls.append(["rblock", a, r, n]); return [0] * n
    for name in ("smbus", "smbus2"):
        m = types.ModuleType(name); m.SMBus = SMBus; sys.modules[name] = m
out = io.StringIO()
with contextlib.redirect_stdout(out):
    from Arm_Lib import Arm_Device
    dev = Arm_Device()
results = []
if mode == "codare":
    for case in json.load(sys.stdin):
        del calls[:]
        state["word"] = case.get("word", 0)
        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            ret = getattr(dev, case["method"])(*case["args"])
        results.append({"calls": list(calls), "ret": ret, "printed": printed.getvalue()})
else:
    with contextlib.redirect_stdout(out):
        results = [dev.Arm_serial_servo_read(i) for i in range(1, 7)]
print("@@JSON@@" + json.dumps(results))
`;

function opsToCalls(ops: I2cOp[]): Call[] {
  return ops.map((op) =>
    op.kind === "block" ? ["block", BOARD_ADDRESS, op.reg, op.bytes] : ["byte", BOARD_ADDRESS, op.reg, op.value],
  );
}

function buildCases(): Case[] {
  const cases: Case[] = [];
  const six: [number[], number][] = [
    [[90, 90, 90, 90, 90, 90], 1000],
    [[0, 0, 0, 0, 0, 0], 500],
    [[180, 180, 180, 180, 270, 180], 2000],
    [[10.5, 45.25, 135.75, 1, 179, 33.3], 0],
    [[37, 73, 111, 149, 200, 12], 65535],
  ];
  for (const [a, t] of six) {
    cases.push({
      name: `write6 ${JSON.stringify(a)} t=${t}`,
      method: "Arm_serial_servo_write6",
      args: [...a, t],
      expectCalls: opsToCalls(encodeWrite6(a as never, t)),
    });
  }
  for (const id of SERVO_IDS) {
    const angles = [0, 12.5, 37, 90, 180, ...(id === 5 ? [270] : [])];
    for (const a of angles) {
      cases.push({
        name: `write servo ${id} ${a}`,
        method: "Arm_serial_servo_write",
        args: [id, a, 500],
        expectCalls: opsToCalls(encodeWriteServo(id, a, 500)),
      });
    }
  }
  for (const on of [true, false]) {
    cases.push({ name: `torque ${on}`, method: "Arm_serial_set_torque", args: [on ? 1 : 0], expectCalls: opsToCalls(encodeTorque(on)) });
  }
  for (const d of [1, 10, 50]) {
    cases.push({ name: `buzzer on ${d}`, method: "Arm_Buzzer_On", args: [d], expectCalls: opsToCalls(encodeBuzzer(d)) });
  }
  cases.push({ name: "buzzer on (implicit)", method: "Arm_Buzzer_On", args: [], expectCalls: opsToCalls(encodeBuzzer(0xff)) });
  cases.push({ name: "buzzer off", method: "Arm_Buzzer_Off", args: [], expectCalls: opsToCalls(encodeBuzzer(0)) });

  for (const id of SERVO_IDS) {
    const { pulseMin, pulseMax } = servoRange(id as ServoId);
    const pulses = new Set([0, 1, pulseMin - 1, pulseMin, pulseMin + 1, pulseMax - 1, pulseMax, pulseMax + 1, 65535]);
    for (let p = pulseMin; p <= pulseMax; p += 23) pulses.add(p);
    for (const p of pulses) {
      const reg = REG.SERVO_READ + id;
      cases.push({
        name: `read servo ${id} puls ${p}`,
        method: "Arm_serial_servo_read",
        args: [id],
        word: swapWord(p),
        expectCalls: [["byte", BOARD_ADDRESS, reg, 0], ["rword", BOARD_ADDRESS, reg]],
        expectReturn: decodeRead(id as ServoId, swapWord(p)),
      });
    }
  }
  return cases;
}

function runPython(mode: string, input: string): unknown {
  const r = spawnSync(values.python, ["-c", PY, mode, values.armlib], { input, encoding: "utf8", maxBuffer: 64 << 20 });
  if (r.error) throw new Error(`nu pot rula ${values.python}: ${r.error.message}`);
  const line = r.stdout.split("\n").find((l) => l.startsWith("@@JSON@@"));
  if (r.status !== 0 || !line) {
    throw new Error(`Python a eșuat (cod ${r.status}):\n${r.stderr || r.stdout}\n` +
      "Verificați că Arm_Lib e instalat (pip3 show -f Arm_Lib) sau dați --armlib DIR.");
  }
  return JSON.parse(line.slice("@@JSON@@".length));
}

async function codare(): Promise<boolean> {
  const cases = buildCases();
  const results = runPython("codare", JSON.stringify(cases)) as { calls: Call[]; ret: unknown; printed: string }[];
  let bad = 0;
  cases.forEach((c, i) => {
    const r = results[i]!;
    const callsOk = JSON.stringify(r.calls) === JSON.stringify(c.expectCalls);
    const retOk = !("expectReturn" in c) || JSON.stringify(r.ret ?? null) === JSON.stringify(c.expectReturn);
    if (!callsOk || !retOk) {
      bad++;
      console.log(`DIFERIT  ${c.name}`);
      console.log(`   Arm_Lib: ${JSON.stringify(r.calls)}${"expectReturn" in c ? ` -> ${JSON.stringify(r.ret)}` : ""}`);
      console.log(`   TS:      ${JSON.stringify(c.expectCalls)}${"expectReturn" in c ? ` -> ${JSON.stringify(c.expectReturn)}` : ""}`);
      if (r.printed.trim()) console.log(`   Arm_Lib a afișat: ${r.printed.trim()}`);
    }
  });
  console.log(`\n${cases.length - bad}/${cases.length} cazuri identice cu Arm_Lib.`);
  return bad === 0;
}

async function citire(): Promise<boolean> {
  const arm = await openI2cArm();
  let ts: (number | null)[];
  try {
    ts = await arm.readAll();
  } finally {
    await arm.close();
  }
  const py = runPython("citire", "") as (number | null)[];
  let ok = true;
  console.log("servo   TS    Arm_Lib");
  SERVO_IDS.forEach((id, i) => {
    const a = ts[i] ?? null;
    const b = py[i] ?? null;
    const same = a !== null && b !== null && Math.abs(a - b) <= 1;
    if (!same) ok = false;
    console.log(`  ${id}   ${String(a).padStart(4)}   ${String(b).padStart(4)}   ${same ? "OK" : "DIFERIT"}`);
  });
  console.log(ok ? "\nCitirile coincid (±1° între două citiri succesive)." : "\nCitirile diferă.");
  return ok;
}

const mode = positionals[0] ?? "codare";
const run = mode === "citire" ? citire : mode === "codare" ? codare : null;
if (!run) {
  console.error("Utilizare: npm run compara:armlib -- <codare|citire> [--armlib DIR] [--python python3]");
  process.exit(2);
}
run().then(
  (ok) => process.exit(ok ? 0 : 1),
  (e: unknown) => {
    console.error(e instanceof Error ? e.message : e);
    process.exit(1);
  },
);
