/*
 * Meteosis MTS-01 radiosonde decoder - core logic (no UI).
 *
 * Input : audio of the FM discriminator output (what SDR# produces in NFM mode).
 * Signal: 2-FSK, ~1200 baud, ~1 s burst about every 2 s.
 * Frame : [alternating preamble] [sync 1101000010101110] [6 bits gap]
 *         [8-bit ASCII bytes, MSB first]:
 *         ...,YYMMDDHHMMSS,<counter>,<lat>,<lon>,<alt>,<...>   (UTC time)
 */
(function (root) {
  const BAUD = 1200;
  const SYNC = "1101000010101110";
  const SYNC_GAP = 6;
  const LEVEL_WINDOW_SYMBOLS = 16;

  // Centered moving average (prefix sums)
  function movingAverage(x, w) {
    const n = x.length, out = new Float32Array(n), pre = new Float64Array(n + 1);
    for (let i = 0; i < n; i++) pre[i + 1] = pre[i] + x[i];
    const h = Math.floor(w / 2);
    for (let i = 0; i < n; i++) {
      const a = Math.max(0, i - h), b = Math.min(n, i + h + 1);
      out[i] = (pre[b] - pre[a]) / (b - a);
    }
    return out;
  }

  // Running midpoint between the two FSK levels (max+min over a window)
  function levelMidpoint(x, w) {
    const n = x.length, out = new Float32Array(n), h = Math.floor(w / 2);
    for (let i = 0; i < n; i++) {
      const a = Math.max(0, i - h), b = Math.min(n, i + h + 1);
      let mx = -Infinity, mn = Infinity;
      for (let j = a; j < b; j++) { const v = x[j]; if (v > mx) mx = v; if (v < mn) mn = v; }
      out[i] = (mx + mn) / 2;
    }
    return out;
  }

  // Slice at a sampling phase (samples) -> bits and confidence
  function sliceBits(x, mid, T, phase) {
    const n = x.length, bits = [], conf = [];
    const w = Math.max(1, Math.round(T / 4));
    for (let c = phase + T / 2; c + 1 < n; c += T) {
      const ci = Math.round(c);
      let s = 0, k = 0;
      for (let j = ci - w; j <= ci + w; j++) if (j >= 0 && j < n) { s += x[j] - mid[j]; k++; }
      const v = s / Math.max(1, k);
      bits.push(v > 0 ? 1 : 0);
      conf.push(Math.abs(v));
    }
    return { bits: Uint8Array.from(bits), conf: Float32Array.from(conf) };
  }

  function findSync(bits) {
    let best = null;
    for (const invert of [false, true]) {
      const target = [...SYNC].map(c => (invert ? 1 - +c : +c));
      for (let i = 0; i + 16 <= bits.length; i++) {
        let err = 0;
        for (let k = 0; k < 16 && err <= 1; k++) if (bits[i + k] !== target[k]) err++;
        if (err <= 1 && (!best || err < best.err)) best = { pos: i, err, invert };
      }
    }
    return best;
  }

  function bitsToText(bits) {
    let out = "";
    for (let j = 0; j + 8 <= bits.length; j += 8) {
      let v = 0;
      for (let k = 0; k < 8; k++) v = (v << 1) | bits[j + k];
      out += v >= 32 && v < 127 ? String.fromCharCode(v) : "~";
    }
    return out;
  }

  function parseText(text) {
    // YYMMDDHHMMSS (UTC), counter, lat, lon, alt
    const m = text.match(/(\d{2})(\d{2})(\d{2})(\d{2})(\d{2})(\d{2}),(\d+),(-?\d{1,2}\.\d{4,}),(-?\d{1,3}\.\d{4,}),(-?\d{1,6})/);
    if (!m) return null;
    const yy = +m[1], mo = +m[2], dd = +m[3], hh = +m[4], mi = +m[5], ss = +m[6];
    const lat = parseFloat(m[8]), lon = parseFloat(m[9]);
    if (mo < 1 || mo > 12 || dd < 1 || dd > 31 || hh > 23 || mi > 59 || ss > 59) return null;
    if (Math.abs(lat) > 90 || Math.abs(lon) > 180) return null;
    const pad = n => String(n).padStart(2, "0");
    return {
      utcDate: `20${pad(yy)}-${pad(mo)}-${pad(dd)}`,
      utcTime: `${pad(hh)}:${pad(mi)}:${pad(ss)}`,
      counter: +m[7],
      lat, lon,
      alt: +m[10],
      raw: text
    };
  }

  // Decode one active audio segment. Every sampling phase that finds the sync
  // votes (weighted by confidence) for each payload bit; then parse the result.
  function decodeSegment(x, fs) {
    const T = fs / BAUD;
    const mid = levelMidpoint(x, Math.round(LEVEL_WINDOW_SYMBOLS * T));
    const maxBits = 1400;
    const score1 = new Float64Array(maxBits), score0 = new Float64Array(maxBits);
    let used = 0, maxLen = 0;
    const steps = Math.max(16, Math.round(T * 2));
    for (let s = 0; s < steps; s++) {
      const { bits, conf } = sliceBits(x, mid, T, (s / steps) * T);
      const sy = findSync(bits);
      if (!sy || sy.err > 1) continue;
      used++;
      const start = sy.pos + 16 + SYNC_GAP;
      for (let k = 0; start + k < bits.length && k < maxBits; k++) {
        let v = bits[start + k];
        if (sy.invert) v = 1 - v;
        if (v) score1[k] += conf[start + k]; else score0[k] += conf[start + k];
        if (k + 1 > maxLen) maxLen = k + 1;
      }
    }
    if (used < 3) return null;
    const voted = new Uint8Array(maxLen);
    for (let k = 0; k < maxLen; k++) voted[k] = score1[k] > score0[k] ? 1 : 0;
    const f = parseText(bitsToText(voted));
    return f ? { ...f, phasesUsed: used } : null;
  }

  // ---------- streaming decoder ----------
  class SondeDecoder {
    constructor(fs, onFrame) {
      this.fs = fs;
      this.onFrame = onFrame || (() => {});
      this.keepSec = 6;
      this.buf = new Float32Array(0);
      this.bufStart = 0;          // absolute sample index of buf[0]
      this.total = 0;             // absolute samples received
      this.dc = 0;                // DC tracker
      this.lastAnalysis = 0;
      this.lastDoneEnd = 0;       // absolute end of last processed burst
      this.recent = [];           // recently reported frames
      this.level = 0;
    }

    push(samples) {
      const fs = this.fs;
      const a = 1 - Math.exp(-2 * Math.PI * 5 / fs);  // very slow DC tracker
      const add = new Float32Array(samples.length);
      let sq = 0;
      for (let i = 0; i < samples.length; i++) {
        this.dc += a * (samples[i] - this.dc);
        add[i] = samples[i] - this.dc;
        sq += add[i] * add[i];
      }
      this.level = Math.sqrt(sq / Math.max(1, samples.length));

      const maxLen = Math.round(this.keepSec * fs);
      const joined = new Float32Array(Math.min(maxLen, this.buf.length + add.length));
      const oldKeep = Math.max(0, joined.length - add.length);
      joined.set(this.buf.subarray(this.buf.length - oldKeep), 0);
      joined.set(add.subarray(add.length - (joined.length - oldKeep)), oldKeep);
      this.bufStart = this.total + add.length - joined.length;
      this.buf = joined;
      this.total += add.length;

      if (this.total - this.lastAnalysis >= 0.25 * fs) {
        this.lastAnalysis = this.total;
        this.analyse();
      }
    }

    analyse() {
      const fs = this.fs, x = this.buf;
      if (x.length < fs * 0.5) return;
      const ma = movingAverage(x, Math.round(fs / BAUD));
      const blk = Math.round(0.02 * fs);
      const nblk = Math.floor(x.length / blk);
      const active = new Uint8Array(nblk);
      for (let b = 0; b < nblk; b++) {
        let pm = 0, pr = 0;
        for (let i = b * blk; i < (b + 1) * blk; i++) { pm += ma[i] * ma[i]; pr += x[i] * x[i]; }
        active[b] = pr > 1e-9 * blk && pm / pr > 0.3 ? 1 : 0;
      }
      const tailBlocks = Math.round(0.25 * fs / blk);
      let b = 0;
      while (b < nblk) {
        if (!active[b]) { b++; continue; }
        let e = b;
        while (e < nblk && active[e]) e++;
        const sAbs = this.bufStart + b * blk, eAbs = this.bufStart + e * blk;
        const complete = e < nblk - tailBlocks;
        if (complete && (eAbs - sAbs) / fs > 0.4 && eAbs > this.lastDoneEnd) {
          const pad = Math.round(0.05 * fs);
          const s0 = Math.max(0, b * blk - pad), e0 = Math.min(x.length, e * blk + pad);
          const f = decodeSegment(x.subarray(s0, e0), fs);
          if (f) {
            const t = this.bufStart + s0;
            const dup = this.recent.some(q => q.raw === f.raw || (q.t > 0 && Math.abs(q.t - t) < fs * 1.5 && q.lat === f.lat));
            if (!dup) {
              this.recent.push({ raw: f.raw, t, lat: f.lat });
              if (this.recent.length > 30) this.recent.shift();
              this.onFrame({ ...f, receivedAt: new Date(), audioTime: t / fs });
            }
          }
          this.lastDoneEnd = eAbs;
        }
        b = e;
      }
    }
  }

  const api = { SondeDecoder, decodeSegment, parseText, SYNC };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.MtsDecoder = api;
})(typeof window !== "undefined" ? window : globalThis);
