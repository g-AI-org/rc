import { describe, it, expect } from "vitest";
import {
  MemoryArtifactStore,
  CamSdArtifactStore,
  computeRetentionScore,
  sha256HexSync,
} from "./artifactStore";
import {
  compileBalaXProgram,
  BALAX_PROGRAM_TEMPLATES,
} from "./balaxProgramCompiler";

describe("artifactStore & Cost-Aware W-TinyLFU", () => {
  it("computes exact NIST SHA-256('abc') synchronously", () => {
    const bytes = new TextEncoder().encode("abc");
    expect(sha256HexSync(bytes)).toBe(
      "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
    );
  });

  it("evicts lowest W-TinyLFU score and never evicts pinned or hot items", async () => {
    const store = new MemoryArtifactStore(3000);
    const now = Date.now();
    await store.put({
      id: "pinned",
      name: "pinned.bin",
      kind: "firmware",
      sha256: "a".repeat(64),
      rawSize: 1000,
      createdAt: now - 100_000,
      lastUsedAt: now - 100_000,
      refetchCost: 1,
      pinned: true,
      source: "local",
      bytes: new Uint8Array(1000),
    });
    await store.put({
      id: "hot-gh",
      name: "stable.bin",
      kind: "firmware",
      sha256: "b".repeat(64),
      rawSize: 1000,
      createdAt: now - 10_000,
      lastUsedAt: now - 10_000,
      refetchCost: 4,
      pinned: false,
      source: "github",
      bytes: new Uint8Array(1000),
    });
    await store.put({
      id: "cold",
      name: "cold.bin",
      kind: "firmware",
      sha256: "c".repeat(64),
      rawSize: 1000,
      createdAt: now - 72 * 3600 * 1000,
      lastUsedAt: now - 72 * 3600 * 1000,
      refetchCost: 1,
      pinned: false,
      source: "local",
      bytes: new Uint8Array(1000),
    });
    // Touch hot item
    await store.get("b".repeat(64));
    await store.get("b".repeat(64));

    // Insert 4th 1000B artifact -> must evict cold ("c"), keeping pinned ("a") and hot ("b")
    await store.put({
      id: "new",
      name: "new.bin",
      kind: "firmware",
      sha256: "d".repeat(64),
      rawSize: 1000,
      createdAt: now,
      lastUsedAt: now,
      refetchCost: 1,
      pinned: false,
      source: "local",
      bytes: new Uint8Array(1000),
    });

    const items = await store.list();
    const shas = items.map((i) => i.sha256);
    expect(shas).toContain("a".repeat(64));
    expect(shas).toContain("b".repeat(64));
    expect(shas).toContain("d".repeat(64));
    expect(shas).not.toContain("c".repeat(64));
    expect(computeRetentionScore(items.find((i) => i.pinned)!)).toBe(Infinity);
  });

  it("handles offline ESP32-CAM SD gracefully without throwing", async () => {
    const cam = new CamSdArtifactStore("http://127.0.0.1:1");
    const st = await cam.status();
    expect(st.available).toBe(false);
    expect(await cam.list()).toEqual([]);
  });
});

describe("balaxProgramCompiler", () => {
  it("compiles all built-in templates into valid BXP1 bytecode", () => {
    for (const tpl of BALAX_PROGRAM_TEMPLATES) {
      const res = compileBalaXProgram(tpl.source);
      expect(res.ok, `${tpl.id}: ${res.error}`).toBe(true);
      expect(res.bytes).toBeDefined();
      expect(res.bytes!.length).toBeGreaterThan(10);
      expect(String.fromCharCode(...res.bytes!.slice(0, 4))).toBe("BXP1");
    }
  });

  it("reports line-numbered syntax errors cleanly", () => {
    const bad = compileBalaXProgram("let x = 1\nunknown_fn(123)\n");
    expect(bad.ok).toBe(false);
    expect(bad.line).toBe(2);
    expect(bad.error).toContain("Unknown function");
  });
});
