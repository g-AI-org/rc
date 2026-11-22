// Shared Browser-First Artifact Storage, GitHub Releases OTA Fetcher, Optional ESP32-CAM SD
// Client, and Cost-Aware W-TinyLFU Cache Eviction Engine for BalaX & companion apps.
// Works 100% offline via IndexedDB with zero ESP32-CAM hardware required.

export type ArtifactKind = "firmware" | "stub" | "program" | "recording";
export type ArtifactSource = "github" | "local" | "cache" | "cam_sd" | "upload";

export interface BalaXArtifactMeta {
  id: string;
  name: string;
  kind: ArtifactKind;
  version?: string;
  channel?: "stable" | "dev" | "local" | "custom";
  sha256: string;
  rawSize: number;
  compressedSize?: number;
  createdAt: number;
  lastUsedAt: number;
  accessCount: number;
  freqDecayed: number;
  refetchCost: number; // higher = more expensive to re-download/rebuild (e.g. GitHub=4, Local=1, Program=2)
  pinned: boolean;
  source: ArtifactSource;
}

export interface BalaXArtifact extends BalaXArtifactMeta {
  bytes: Uint8Array;
}

export interface ArtifactStore {
  readonly id: string;
  readonly label: string;
  isAvailable(): Promise<boolean>;
  list(kind?: ArtifactKind): Promise<BalaXArtifactMeta[]>;
  get(sha256: string): Promise<BalaXArtifact | null>;
  put(artifact: Omit<BalaXArtifact, "accessCount" | "freqDecayed" | "lastUsedAt"> & {
    accessCount?: number;
    freqDecayed?: number;
    lastUsedAt?: number;
  }): Promise<BalaXArtifactMeta>;
  setPinned(sha256: string, pinned: boolean): Promise<boolean>;
  delete(sha256: string): Promise<void>;
  clearUnpinned(): Promise<number>;
}

// Pure-TS SHA-256 fallback so verification works over plain LAN HTTP (`http://balax-remote.local:5180`)
// where browsers disable `crypto.subtle` outside secure contexts.
const SHA_K = new Uint32Array([
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
]);

export function sha256HexSync(data: Uint8Array): string {
  const len = data.length;
  const bitLenHi = Math.floor((len * 8) / 0x100000000);
  const bitLenLo = (len * 8) >>> 0;
  const paddedLen = ((len + 9 + 63) >> 6) << 6;
  const buf = new Uint8Array(paddedLen);
  buf.set(data);
  buf[len] = 0x80;
  const view = new DataView(buf.buffer);
  view.setUint32(paddedLen - 8, bitLenHi, false);
  view.setUint32(paddedLen - 4, bitLenLo, false);

  let h0 = 0x6a09e667, h1 = 0xbb67ae85, h2 = 0x3c6ef372, h3 = 0xa54ff53a;
  let h4 = 0x510e527f, h5 = 0x9b05688c, h6 = 0x1f83d9ab, h7 = 0x5be0cd19;
  const w = new Uint32Array(64);
  const rotr = (x: number, n: number) => (x >>> n) | (x << (32 - n));

  for (let off = 0; off < paddedLen; off += 64) {
    for (let i = 0; i < 16; i++) w[i] = view.getUint32(off + i * 4, false);
    for (let i = 16; i < 64; i++) {
      const s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >>> 3);
      const s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >>> 10);
      w[i] = (w[i - 16] + s0 + w[i - 7] + s1) >>> 0;
    }
    let a = h0, b = h1, c = h2, d = h3, e = h4, f = h5, g = h6, h = h7;
    for (let i = 0; i < 64; i++) {
      const S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const ch = (e & f) ^ (~e & g);
      const t1 = (h + S1 + ch + SHA_K[i] + w[i]) >>> 0;
      const S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const maj = (a & b) ^ (a & c) ^ (b & c);
      const t2 = (S0 + maj) >>> 0;
      h = g; g = f; f = e; e = (d + t1) >>> 0;
      d = c; c = b; b = a; a = (t1 + t2) >>> 0;
    }
    h0 = (h0 + a) >>> 0; h1 = (h1 + b) >>> 0; h2 = (h2 + c) >>> 0; h3 = (h3 + d) >>> 0;
    h4 = (h4 + e) >>> 0; h5 = (h5 + f) >>> 0; h6 = (h6 + g) >>> 0; h7 = (h7 + h) >>> 0;
  }
  return [h0, h1, h2, h3, h4, h5, h6, h7].map((v) => v.toString(16).padStart(8, "0")).join("");
}

export async function sha256Hex(data: Uint8Array): Promise<string> {
  if (typeof crypto !== "undefined" && crypto.subtle) {
    try {
      const copy = new Uint8Array(data.byteLength);
      copy.set(data);
      const digest = await crypto.subtle.digest("SHA-256", copy.buffer);
      return Array.from(new Uint8Array(digest))
        .map((b) => b.toString(16).padStart(2, "0"))
        .join("");
    } catch {
      // Fall back to pure-TS implementation
    }
  }
  return sha256HexSync(data);
}

// --- Cost-Aware W-TinyLFU Eviction Policy ---
// Protects pinned entries, frequently-used artifacts, expensive-to-refetch GitHub images,
// and compact mini-programs while cleanly evicting stale large cold artifacts.
export function computeRetentionScore(meta: BalaXArtifactMeta, nowMs: number = Date.now()): number {
  if (meta.pinned) return Number.POSITIVE_INFINITY;
  const ageHours = Math.max(0.05, (nowMs - meta.lastUsedAt) / (1000 * 60 * 60));
  const sizeKb = Math.max(1, (meta.compressedSize ?? meta.rawSize) / 1024);
  const freq = Math.max(0.25, meta.freqDecayed || 1);
  const cost = Math.max(0.5, meta.refetchCost || 1);
  return (freq * cost) / (Math.pow(ageHours, 0.75) * Math.sqrt(sizeKb));
}

export function selectEvictionVictims(
  entries: BalaXArtifactMeta[],
  incomingBytes: number,
  budgetBytes: number,
  nowMs: number = Date.now(),
): BalaXArtifactMeta[] {
  const totalBytes = entries.reduce((sum, e) => sum + e.rawSize, 0);
  let over = totalBytes + incomingBytes - budgetBytes;
  if (over <= 0) return [];
  const candidates = entries
    .filter((e) => !e.pinned)
    .map((e) => ({ meta: e, score: computeRetentionScore(e, nowMs) }))
    .sort((a, b) => a.score - b.score);
  const victims: BalaXArtifactMeta[] = [];
  for (const c of candidates) {
    if (over <= 0) break;
    victims.push(c.meta);
    over -= c.meta.rawSize;
  }
  return victims;
}

export function decayFrequency(freq: number, halfLifeHours: number, elapsedHours: number): number {
  if (elapsedHours <= 0) return Math.max(0.25, freq);
  return Math.max(0.25, freq * Math.pow(0.5, elapsedHours / Math.max(1, halfLifeHours)));
}

// --- In-Memory Reference Store (used by tests & non-IDB environments) ---
export class MemoryArtifactStore implements ArtifactStore {
  readonly id = "memory";
  readonly label = "Memory Cache";
  private map = new Map<string, BalaXArtifact>();
  private readonly budgetBytes: number;

  constructor(budgetBytes = 32 * 1024 * 1024) {
    this.budgetBytes = budgetBytes;
  }

  async isAvailable(): Promise<boolean> {
    return true;
  }

  async list(kind?: ArtifactKind): Promise<BalaXArtifactMeta[]> {
    const out: BalaXArtifactMeta[] = [];
    for (const a of this.map.values()) {
      if (!kind || a.kind === kind) {
        const { bytes: _b, ...meta } = a;
        out.push(meta);
      }
    }
    return out.sort((a, b) => b.lastUsedAt - a.lastUsedAt);
  }

  async get(sha256: string): Promise<BalaXArtifact | null> {
    const entry = this.map.get(sha256);
    if (!entry) return null;
    const now = Date.now();
    const elapsedH = Math.max(0, (now - entry.lastUsedAt) / (1000 * 60 * 60));
    entry.freqDecayed = decayFrequency(entry.freqDecayed, 24, elapsedH) + 1;
    entry.accessCount += 1;
    entry.lastUsedAt = now;
    return { ...entry, bytes: new Uint8Array(entry.bytes) };
  }

  async put(
    artifact: Omit<BalaXArtifact, "accessCount" | "freqDecayed" | "lastUsedAt"> & {
      accessCount?: number;
      freqDecayed?: number;
      lastUsedAt?: number;
    },
  ): Promise<BalaXArtifactMeta> {
    const now = artifact.lastUsedAt ?? Date.now();
    const existing = this.map.get(artifact.sha256);
    if (existing) {
      existing.name = artifact.name || existing.name;
      existing.version = artifact.version ?? existing.version;
      existing.channel = artifact.channel ?? existing.channel;
      existing.pinned = existing.pinned || artifact.pinned;
      existing.freqDecayed += 1;
      existing.accessCount += 1;
      existing.lastUsedAt = now;
      const { bytes: _b, ...meta } = existing;
      return meta;
    }

    const victims = selectEvictionVictims(await this.list(), artifact.rawSize, this.budgetBytes, now);
    for (const v of victims) {
      this.map.delete(v.sha256);
    }

    const full: BalaXArtifact = {
      ...artifact,
      bytes: new Uint8Array(artifact.bytes),
      lastUsedAt: now,
      accessCount: artifact.accessCount ?? 1,
      freqDecayed: artifact.freqDecayed ?? 1,
    };
    this.map.set(full.sha256, full);
    const { bytes: _b, ...meta } = full;
    return meta;
  }

  async setPinned(sha256: string, pinned: boolean): Promise<boolean> {
    const e = this.map.get(sha256);
    if (!e) return false;
    e.pinned = pinned;
    return true;
  }

  async delete(sha256: string): Promise<void> {
    this.map.delete(sha256);
  }

  async clearUnpinned(): Promise<number> {
    let removed = 0;
    for (const [k, v] of this.map.entries()) {
      if (!v.pinned) {
        this.map.delete(k);
        removed += 1;
      }
    }
    return removed;
  }
}

// --- Browser IndexedDB Artifact Store (Tier-1 Persistent Offline Cache) ---
const IDB_NAME = "balax_artifact_cache_v1";
const IDB_STORE = "artifacts";

function openArtifactDb(): Promise<IDBDatabase> {
  return new Promise((resolve, reject) => {
    const req = indexedDB.open(IDB_NAME, 1);
    req.onupgradeneeded = () => {
      const db = req.result;
      if (!db.objectStoreNames.contains(IDB_STORE)) {
        db.createObjectStore(IDB_STORE, { keyPath: "sha256" });
      }
    };
    req.onsuccess = () => resolve(req.result);
    req.onerror = () => reject(req.error ?? new Error("IndexedDB open failed"));
  });
}

export class IndexedDbArtifactStore implements ArtifactStore {
  readonly id = "browser_cache";
  readonly label = "Browser Cache (IndexedDB)";
  private readonly budgetBytes: number;

  constructor(budgetBytes = 64 * 1024 * 1024) {
    this.budgetBytes = budgetBytes;
  }

  async isAvailable(): Promise<boolean> {
    return typeof indexedDB !== "undefined";
  }

  async list(kind?: ArtifactKind): Promise<BalaXArtifactMeta[]> {
    if (!(await this.isAvailable())) return [];
    const db = await openArtifactDb();
    return new Promise((resolve, reject) => {
      const tx = db.transaction(IDB_STORE, "readonly");
      const req = tx.objectStore(IDB_STORE).getAll();
      req.onsuccess = () => {
        const all = (req.result as BalaXArtifact[])
          .filter((a) => !kind || a.kind === kind)
          .map(({ bytes: _b, ...meta }) => meta)
          .sort((a, b) => b.lastUsedAt - a.lastUsedAt);
        resolve(all);
      };
      req.onerror = () => reject(req.error);
    });
  }

  async get(sha256: string): Promise<BalaXArtifact | null> {
    if (!(await this.isAvailable())) return null;
    const db = await openArtifactDb();
    const record = await new Promise<BalaXArtifact | undefined>((resolve, reject) => {
      const tx = db.transaction(IDB_STORE, "readonly");
      const req = tx.objectStore(IDB_STORE).get(sha256);
      req.onsuccess = () => resolve(req.result as BalaXArtifact | undefined);
      req.onerror = () => reject(req.error);
    });
    if (!record) return null;
    const now = Date.now();
    const elapsedH = Math.max(0, (now - record.lastUsedAt) / (1000 * 60 * 60));
    record.freqDecayed = decayFrequency(record.freqDecayed || 1, 24, elapsedH) + 1;
    record.accessCount = (record.accessCount || 0) + 1;
    record.lastUsedAt = now;
    await new Promise<void>((resolve, reject) => {
      const tx = db.transaction(IDB_STORE, "readwrite");
      tx.objectStore(IDB_STORE).put(record);
      tx.oncomplete = () => resolve();
      tx.onerror = () => reject(tx.error);
    });
    return { ...record, bytes: record.bytes instanceof Uint8Array ? record.bytes : new Uint8Array(record.bytes) };
  }

  async put(
    artifact: Omit<BalaXArtifact, "accessCount" | "freqDecayed" | "lastUsedAt"> & {
      accessCount?: number;
      freqDecayed?: number;
      lastUsedAt?: number;
    },
  ): Promise<BalaXArtifactMeta> {
    const db = await openArtifactDb();
    const existingList = await this.list();
    const existing = existingList.find((e) => e.sha256 === artifact.sha256);
    const now = artifact.lastUsedAt ?? Date.now();
    if (!existing) {
      const victims = selectEvictionVictims(existingList, artifact.rawSize, this.budgetBytes, now);
      if (victims.length > 0) {
        await new Promise<void>((resolve, reject) => {
          const tx = db.transaction(IDB_STORE, "readwrite");
          const store = tx.objectStore(IDB_STORE);
          for (const v of victims) store.delete(v.sha256);
          tx.oncomplete = () => resolve();
          tx.onerror = () => reject(tx.error);
        });
      }
    }

    const full: BalaXArtifact = {
      ...artifact,
      bytes: new Uint8Array(artifact.bytes),
      lastUsedAt: now,
      accessCount: existing ? existing.accessCount + 1 : (artifact.accessCount ?? 1),
      freqDecayed: existing ? existing.freqDecayed + 1 : (artifact.freqDecayed ?? 1),
      pinned: existing ? existing.pinned || artifact.pinned : artifact.pinned,
    };
    await new Promise<void>((resolve, reject) => {
      const tx = db.transaction(IDB_STORE, "readwrite");
      tx.objectStore(IDB_STORE).put(full);
      tx.oncomplete = () => resolve();
      tx.onerror = () => reject(tx.error);
    });
    const { bytes: _b, ...meta } = full;
    return meta;
  }

  async setPinned(sha256: string, pinned: boolean): Promise<boolean> {
    if (!(await this.isAvailable())) return false;
    const db = await openArtifactDb();
    const rec = await new Promise<BalaXArtifact | undefined>((resolve, reject) => {
      const tx = db.transaction(IDB_STORE, "readonly");
      const req = tx.objectStore(IDB_STORE).get(sha256);
      req.onsuccess = () => resolve(req.result as BalaXArtifact | undefined);
      req.onerror = () => reject(req.error);
    });
    if (!rec) return false;
    rec.pinned = pinned;
    await new Promise<void>((resolve, reject) => {
      const tx = db.transaction(IDB_STORE, "readwrite");
      tx.objectStore(IDB_STORE).put(rec);
      tx.oncomplete = () => resolve();
      tx.onerror = () => reject(tx.error);
    });
    return true;
  }

  async delete(sha256: string): Promise<void> {
    if (!(await this.isAvailable())) return;
    const db = await openArtifactDb();
    await new Promise<void>((resolve, reject) => {
      const tx = db.transaction(IDB_STORE, "readwrite");
      tx.objectStore(IDB_STORE).delete(sha256);
      tx.oncomplete = () => resolve();
      tx.onerror = () => reject(tx.error);
    });
  }

  async clearUnpinned(): Promise<number> {
    const entries = await this.list();
    let count = 0;
    for (const e of entries) {
      if (!e.pinned) {
        await this.delete(e.sha256);
        count += 1;
      }
    }
    return count;
  }
}

// --- Optional ESP32-CAM SD Store (gracefully offline until wired!) ---
export interface CamSdStatus {
  available: boolean;
  host: string;
  freeMb?: number;
  totalMb?: number;
  error?: string;
}

export class CamSdArtifactStore implements ArtifactStore {
  readonly id = "cam_sd";
  readonly label = "ESP32-CAM SD";
  private readonly baseUrl: string;

  constructor(baseUrl = "http://balax-cam.local") {
    this.baseUrl = baseUrl.replace(/\/$/, "");
  }

  async status(): Promise<CamSdStatus> {
    try {
      const ctl = new AbortController();
      const timer = setTimeout(() => ctl.abort(), 1500);
      const res = await fetch(`${this.baseUrl}/sd/status`, { signal: ctl.signal });
      clearTimeout(timer);
      if (!res.ok) return { available: false, host: this.baseUrl, error: `HTTP ${res.status}` };
      const j = (await res.json()) as { free_mb?: number; total_mb?: number };
      return { available: true, host: this.baseUrl, freeMb: j.free_mb, totalMb: j.total_mb };
    } catch {
      return { available: false, host: this.baseUrl, error: "Not connected (optional)" };
    }
  }

  async isAvailable(): Promise<boolean> {
    return (await this.status()).available;
  }

  async list(kind?: ArtifactKind): Promise<BalaXArtifactMeta[]> {
    try {
      const ctl = new AbortController();
      const timer = setTimeout(() => ctl.abort(), 2000);
      const res = await fetch(`${this.baseUrl}/sd/list`, { signal: ctl.signal });
      clearTimeout(timer);
      if (!res.ok) return [];
      const items = (await res.json()) as BalaXArtifactMeta[];
      return kind ? items.filter((i) => i.kind === kind) : items;
    } catch {
      return [];
    }
  }

  async get(sha256: string): Promise<BalaXArtifact | null> {
    try {
      const res = await fetch(`${this.baseUrl}/sd/file?sha256=${encodeURIComponent(sha256)}`);
      if (!res.ok) return null;
      const bytes = new Uint8Array(await res.arrayBuffer());
      return {
        id: sha256,
        name: res.headers.get("X-Artifact-Name") ?? sha256.slice(0, 12),
        kind: (res.headers.get("X-Artifact-Kind") as ArtifactKind) ?? "firmware",
        sha256,
        rawSize: bytes.byteLength,
        createdAt: Date.now(),
        lastUsedAt: Date.now(),
        accessCount: 1,
        freqDecayed: 1,
        refetchCost: 1.5,
        pinned: false,
        source: "cam_sd",
        bytes,
      };
    } catch {
      return null;
    }
  }

  async put(artifact: Omit<BalaXArtifact, "accessCount" | "freqDecayed" | "lastUsedAt">): Promise<BalaXArtifactMeta> {
    const copy = new Uint8Array(artifact.bytes.byteLength);
    copy.set(artifact.bytes);
    const res = await fetch(`${this.baseUrl}/sd/file?sha256=${encodeURIComponent(artifact.sha256)}`, {
      method: "PUT",
      headers: {
        "Content-Type": "application/octet-stream",
        "X-Artifact-Name": artifact.name,
        "X-Artifact-Kind": artifact.kind,
      },
      body: copy.buffer,
    });
    if (!res.ok) throw new Error(`CAM SD upload failed: HTTP ${res.status}`);
    return {
      ...artifact,
      lastUsedAt: Date.now(),
      accessCount: 1,
      freqDecayed: 1,
      source: "cam_sd",
    };
  }

  async setPinned(): Promise<boolean> {
    return false;
  }

  async delete(sha256: string): Promise<void> {
    await fetch(`${this.baseUrl}/sd/file?sha256=${encodeURIComponent(sha256)}`, { method: "DELETE" }).catch(() => {});
  }

  async clearUnpinned(): Promise<number> {
    return 0;
  }
}

// --- Offloaded GitHub Releases / Pages Firmware Fetcher (runs in Browser — zero TLS on robot!) ---
export interface GitHubFirmwareManifest {
  channel: "stable" | "dev";
  version: string;
  sha256?: string;
  size?: number;
  url: string;
  publishedAt?: string;
}

export const DEFAULT_GITHUB_FW_BASE = "https://esp32-bot.github.io/BalaX/fw";

export async function fetchGitHubReleaseManifest(
  channel: "stable" | "dev",
  baseUrl: string = DEFAULT_GITHUB_FW_BASE,
): Promise<GitHubFirmwareManifest> {
  const cleanBase = baseUrl.replace(/\/$/, "");
  const manifestUrl = `${cleanBase}/${channel}/manifest.json`;
  const res = await fetch(manifestUrl, { cache: "no-store" });
  if (!res.ok) {
    throw new Error(`Manifest fetch failed (${manifestUrl}): HTTP ${res.status}`);
  }
  const j = (await res.json()) as {
    version?: string;
    sha256?: string;
    size?: number;
    file?: string;
    date?: string;
  };
  const file = j.file || "balax.bin";
  return {
    channel,
    version: j.version || channel,
    sha256: j.sha256?.toLowerCase(),
    size: j.size,
    url: `${cleanBase}/${channel}/${file}`,
    publishedAt: j.date,
  };
}

export async function downloadAndCacheGitHubFirmware(
  channel: "stable" | "dev",
  store: ArtifactStore,
  baseUrl: string = DEFAULT_GITHUB_FW_BASE,
): Promise<{ manifest: GitHubFirmwareManifest; artifact: BalaXArtifact; fromCache: boolean }> {
  const manifest = await fetchGitHubReleaseManifest(channel, baseUrl);
  if (manifest.sha256) {
    const cached = await store.get(manifest.sha256);
    if (cached) {
      return { manifest, artifact: cached, fromCache: true };
    }
  }
  const binRes = await fetch(manifest.url, { cache: "no-store" });
  if (!binRes.ok) {
    throw new Error(`Firmware download failed (${manifest.url}): HTTP ${binRes.status}`);
  }
  const bytes = new Uint8Array(await binRes.arrayBuffer());
  const actualSha = await sha256Hex(bytes);
  if (manifest.sha256 && actualSha !== manifest.sha256) {
    throw new Error(`SHA-256 mismatch! expected=${manifest.sha256.slice(0, 12)} got=${actualSha.slice(0, 12)}`);
  }
  const meta = await store.put({
    id: `gh-${channel}-${manifest.version}`,
    name: `balax-${channel}-${manifest.version}.bin`,
    kind: "firmware",
    version: manifest.version,
    channel,
    sha256: actualSha,
    rawSize: bytes.byteLength,
    createdAt: Date.now(),
    refetchCost: 4.0, // High cost to re-download from internet → protected by W-TinyLFU!
    pinned: false,
    source: "github",
    bytes,
  });
  return { manifest, artifact: { ...meta, bytes }, fromCache: false };
}
