// =====================================================================
//  noTelemetry.js — block MediaPipe Tasks usage telemetry.
//  tasks-vision 1.2.x contains a usage logger that POSTs anonymous usage
//  statistics to https://odml.pa.googleapis.com/v1/log every 60 s. BoneSim
//  never sends video or pose data anywhere; to keep that promise strict we
//  answer that request locally (HTTP 204) without touching the network.
//  The logger treats a non-200 answer as "logging failed" and stops its timer.
//  Must be imported BEFORE vendor/mediapipe/vision_bundle.mjs.
// =====================================================================
const BLOCKED = /^https:\/\/odml\.pa\.googleapis\.com\//;
const g = globalThis;
if (typeof g.fetch === 'function' && !g.fetch.__bonesimNoTelemetry) {
  const orig = g.fetch.bind(g);
  const wrapped = (input, init) => {
    const url = typeof input === 'string' ? input : input?.url ?? String(input);
    if (BLOCKED.test(url)) return Promise.resolve(new Response(null, { status: 204, statusText: 'blocked by BoneSim' }));
    return orig(input, init);
  };
  wrapped.__bonesimNoTelemetry = true;
  g.fetch = wrapped;
}
export const telemetryBlocked = true;
