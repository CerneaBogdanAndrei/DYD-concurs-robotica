// Serverul HTTP local prin care creierul (Python) comandă brațul.
// Contractul complet: docs/CONTRACT_BRAT.md.

import { type IncomingMessage, type Server, type ServerResponse, createServer } from "node:http";
import { CalibError } from "./calib.js";
import { JointLimitError } from "./robot.js";
import { ArgumentError, type ArmService } from "./service.js";

type Body = Record<string, unknown>;
type Handler = (svc: ArmService, body: Body) => Promise<unknown>;

/** Fiecare rută întoarce `result`; pick/place întorc { result: bool, reason? }. */
const ROUTES: Record<string, Handler> = {
  status: async (svc) => svc.status(),
  home: async (svc) => svc.home(),
  go_pose: async (svc, b) => svc.goPose(b.name as string, (b.speed as "fast" | "slow" | undefined) ?? "fast"),
  move_joints: async (svc, b) => svc.moveJoints(b.angles, b.time_ms),
  open_gripper: async (svc) => svc.openGripper(),
  close_gripper: async (svc) => svc.closeGripper(),
  pick: async (svc, b) => svc.pick(b as never),
  place: async (svc, b) => svc.place(b as never),
  read_joints: async (svc) => svc.readJoints(),
  beep: async (svc, b) => svc.beep(b.n ?? 1),
};

const MAX_BODY = 64 * 1024;

function readBody(req: IncomingMessage): Promise<Body> {
  return new Promise((resolve, reject) => {
    let size = 0;
    const chunks: Buffer[] = [];
    req.on("data", (c: Buffer) => {
      size += c.length;
      if (size > MAX_BODY) {
        reject(new ArgumentError("corpul cererii e prea mare"));
        req.destroy();
      } else chunks.push(c);
    });
    req.on("end", () => {
      const text = Buffer.concat(chunks).toString("utf8").trim();
      if (text === "") return resolve({});
      try {
        const v: unknown = JSON.parse(text);
        if (typeof v !== "object" || v === null || Array.isArray(v)) {
          reject(new ArgumentError("corpul trebuie să fie un obiect JSON"));
        } else resolve(v as Body);
      } catch {
        reject(new ArgumentError("JSON invalid"));
      }
    });
    req.on("error", reject);
  });
}

function send(res: ServerResponse, status: number, payload: unknown): void {
  const text = JSON.stringify(payload);
  res.writeHead(status, { "content-type": "application/json; charset=utf-8", "content-length": Buffer.byteLength(text) });
  res.end(text);
}

/** Eroare -> (status HTTP, cod). Codurile sunt documentate în CONTRACT_BRAT.md. */
function classify(e: unknown): [number, string] {
  if (e instanceof ArgumentError) return [400, "argument"];
  if (e instanceof JointLimitError) return [400, "limite"];
  if (e instanceof RangeError) return [400, "limite"];
  if (e instanceof CalibError) return [409, "calibrare"];
  return [500, "hardware"];
}

function describe(name: string, body: Body): string {
  const parts = [body.location, body.cell && `(${(body.cell as number[]).join(",")})`, body.level && `L${body.level}`, body.name];
  return [name, ...parts.filter(Boolean)].join(" ");
}

export function createArmServer(svc: ArmService, log: (msg: string) => void): Server {
  // Brațul face o singură mișcare odată: cererile se execută strict în ordinea sosirii.
  let queue: Promise<unknown> = Promise.resolve();

  return createServer((req, res) => {
    const name = (req.url ?? "").split("?")[0]!.replace(/^\/+|\/+$/g, "");
    const handler = ROUTES[name];
    if (!handler) return send(res, 404, { error: { code: "ruta", message: `rută necunoscută: /${name}` } });
    if (req.method !== "POST" && !(req.method === "GET" && (name === "status" || name === "read_joints"))) {
      return send(res, 405, { error: { code: "metoda", message: `folosiți POST /${name}` } });
    }

    // Corpul se citește imediat; doar execuția așteaptă la coadă.
    const bodyP = readBody(req);
    bodyP.catch(() => undefined);
    const run = async () => {
      const t0 = Date.now();
      let body: Body = {};
      try {
        body = await bodyP;
        const result = await handler(svc, body);
        const secs = ((Date.now() - t0) / 1000).toFixed(1);
        if (name === "pick" || name === "place") {
          const out = result as { ok: boolean; reason?: string };
          log(`${describe(name, body)} -> ${out.ok ? "OK" : `NU (${out.reason})`} ${secs}s`);
          send(res, 200, out.ok ? { result: true } : { result: false, reason: out.reason });
        } else {
          if (name !== "status" && name !== "read_joints") log(`${describe(name, body)} ${secs}s`);
          send(res, 200, { result: result ?? null });
        }
      } catch (e) {
        const [status, code] = classify(e);
        const message = e instanceof Error ? e.message : String(e);
        log(`${describe(name, body)} -> EROARE ${code}: ${message}`);
        send(res, status, { error: { code, message } });
      }
    };
    queue = queue.then(run, run);
  });
}
