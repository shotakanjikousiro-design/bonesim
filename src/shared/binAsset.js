// =====================================================================
//  binAsset.js — load a binary asset (GLB, .task, .wasm) as Uint8Array.
//
//  1) Normal: fetch(url)  (local server / start.bat, GitHub Pages, …)
//  2) Fallback: import(url + '.b64.js')  — a JS module whose default export
//     is the file as base64. Used by the hosted Claude page build
//     (tools/build_artifact.py), which can only serve text files.
// =====================================================================
const cache = new Map();

function b64ToBytes(s) {
  if (typeof Uint8Array.fromBase64 === 'function') return Uint8Array.fromBase64(s);
  const bin = atob(s); const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

export async function loadBinary(url, { minBytes = 1000 } = {}) {
  const abs = new URL(url, document.baseURI).href;
  if (cache.has(abs)) return cache.get(abs);
  let bytes = null;
  try {
    const r = await fetch(abs);
    if (r.ok) {
      const b = new Uint8Array(await r.arrayBuffer());
      if (b.byteLength >= minBytes) bytes = b;
    }
  } catch { /* fall through to the base64 module */ }
  if (!bytes) {
    const mod = await import(/* @vite-ignore */ `${abs}.b64.js`);
    bytes = b64ToBytes(mod.default);
  }
  cache.set(abs, bytes);
  return bytes;
}

/** Same-origin check: base64 fallback only makes sense for our own files. */
export const isOwnFile = (url) => new URL(url, document.baseURI).origin === location.origin;
