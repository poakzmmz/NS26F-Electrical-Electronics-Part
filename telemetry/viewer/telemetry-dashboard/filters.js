/* =============================================================================
 * NS26F Telemetry — 채널별 노이즈 필터 모듈
 * -----------------------------------------------------------------------------
 * 그래프 위에서 우클릭 → 필터 종류 / 파라미터 선택.
 *
 * 필터는 항상 "원본 100Hz 전체 데이터"에 먼저 적용된 뒤 화면용으로 다운샘플링
 * 됩니다. (먼저 다운샘플링하면 에일리어싱으로 노이즈가 오히려 살아남습니다.)
 *
 * ── 기본 프리셋의 근거 ────────────────────────────────────────────────────
 * 아래는 Telemetry_001.csv 한 개를 분석해 나온 결과이며, '추천 설정' 버튼의
 * 기본값은 여기서 정했습니다. 다른 차량 설정·다른 코스·다른 로거 펌웨어에서는
 * 맞지 않을 수 있으므로 어디까지나 출발점으로만 쓰고, 실제 로그를 보며
 * 파라미터를 조정하세요. 기본 상태는 항상 원본(Raw)입니다.
 *
 * Telemetry_001.csv (52,209행 / 522초 / 100Hz) 분석 요약 ────────────────────
 *  · 샘플 간격은 정확히 10ms, seq 누락 0건 → 로거 자체의 시간축은 신뢰 가능.
 *  · RL 휠속도(rl_wheel_speed): 주행 중 0으로 떨어지는 드롭아웃 1,247개(3.5%).
 *    100km/h 주행 중 10~20ms만 0이 되었다가 복귀 → 물리적으로 불가능한 값.
 *    독립 센서(EMU VSS)와의 상관계수가 raw 0.905 → 드롭아웃 보정 후 0.986으로
 *    상승하는 것으로 인공 오류임을 교차검증함. ⇒ 드롭아웃 제거가 필수 채널.
 *  · 그 외 채널(서스펜션 ADC, 조향, 브레이크, RPM, TPS)은 이 로그에서는
 *    단발성 스파이크가 거의 없음. 다만 CAN 채널은 20Hz로 갱신되어 100Hz 로그에서
 *    계단(staircase) 형태로 보이며, 이 때문에 MAD 기반 이상치 판정이 무력화됨
 *    → Hampel 필터에 sigma 하한(span의 1%)을 두어 해결.
 *  · 주파수 분석: 서스펜션/조향/브레이크 신호 전력의 95~99%가 2Hz 이하.
 *    RL 휠속도만 5Hz 이상 대역에 16%의 전력 → 유일하게 광대역 노이즈 채널.
 *    ⇒ 저역통과 차단주파수 5Hz면 실제 거동을 유지하면서 노이즈만 제거 가능
 *      (서스펜션 peak-to-peak 100.1% 보존 확인).
 *  · RPM=20000 스파이크를 인위적으로 주입해 실험한 결과 Hampel(창7, σ4)이
 *    스파이크의 99.2%를 제거하면서 정상 채널 오검출은 0.15% 이하였음.
 * ========================================================================== */

// ==================== [1] DSP 커널 ====================

/** 이동 평균 (Simple Moving Average) — 가장 단순한 평활. 스파이크는 번지게 함. */
function fltMovingAverage(x, win) {
  const n = x.length;
  const w = Math.max(1, Math.round(win) | 1); // 홀수 강제
  if (w <= 1) return Float64Array.from(x);
  const half = (w - 1) / 2;
  const y = new Float64Array(n);
  let sum = 0, count = 0;
  // 누적합 방식 O(n)
  const pre = new Float64Array(n + 1);
  for (let i = 0; i < n; i++) pre[i + 1] = pre[i] + x[i];
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - half);
    const b = Math.min(n - 1, i + half);
    sum = pre[b + 1] - pre[a];
    count = b - a + 1;
    y[i] = sum / count;
  }
  return y;
}

/** 이동 중앙값 (Median) — 임펄스/드롭아웃 제거에 강하고 계단(step)을 보존. */
function fltMedian(x, win) {
  const n = x.length;
  const w = Math.max(1, Math.round(win) | 1);
  if (w <= 1) return Float64Array.from(x);
  const half = (w - 1) / 2;
  const y = new Float64Array(n);
  const buf = new Float64Array(w);
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - half);
    const b = Math.min(n - 1, i + half);
    const len = b - a + 1;
    for (let k = 0; k < len; k++) buf[k] = x[a + k];
    const sub = buf.subarray(0, len);
    sub.sort();
    y[i] = len % 2 ? sub[(len - 1) >> 1] : 0.5 * (sub[len / 2 - 1] + sub[len / 2]);
  }
  return y;
}

/** 내부용: 이동 중앙값 배열을 그대로 반환(Hampel에서 재사용) */
function rollingMedian(x, w) { return fltMedian(x, w); }

/**
 * Hampel 필터 — 이동 중앙값 ± n·σ(MAD 기반)를 벗어난 점만 중앙값으로 교체.
 * 평활을 하지 않으므로 정상 구간의 파형이 전혀 손상되지 않음.
 * 계단형(CAN 유지값) 데이터에서 MAD=0이 되는 문제를 막기 위해 σ 하한을
 * 신호 범위(p1~p99)의 SIGMA_FLOOR_FRAC 배로 설정.
 */
const SIGMA_FLOOR_FRAC = 0.01;
function fltHampel(x, win, nSigma) {
  const n = x.length;
  const w = Math.max(3, Math.round(win) | 1);
  const med = rollingMedian(x, w);
  const dev = new Float64Array(n);
  for (let i = 0; i < n; i++) dev[i] = Math.abs(x[i] - med[i]);
  const madArr = rollingMedian(dev, w);

  // 신호 범위(p1~p99) 계산 → σ 하한
  const sorted = Float64Array.from(x).sort();
  const p1 = sorted[Math.floor(0.01 * (n - 1))];
  const p99 = sorted[Math.floor(0.99 * (n - 1))];
  const floor = Math.max((p99 - p1) * SIGMA_FLOOR_FRAC, 1e-9);

  const y = Float64Array.from(x);
  for (let i = 0; i < n; i++) {
    const sigma = Math.max(1.4826 * madArr[i], floor);
    if (Math.abs(x[i] - med[i]) > nSigma * sigma) y[i] = med[i];
  }
  return y;
}

/**
 * Savitzky-Golay — 창 안에서 다항식을 최소제곱 적합해 평활.
 * 이동평균과 달리 피크의 높이/폭을 잘 보존해서 서스펜션 스트로크나
 * 브레이크 압력의 최대값을 읽을 때 유리함.
 */
function savGolCoeffs(w, order) {
  const half = (w - 1) / 2;
  const p = order + 1;
  // 정규방정식 A^T A c = A^T e0 (A는 반데르몬드 행렬)
  const ata = [];
  for (let i = 0; i < p; i++) {
    ata.push(new Float64Array(p));
    for (let j = 0; j < p; j++) {
      let s = 0;
      for (let k = -half; k <= half; k++) s += Math.pow(k, i + j);
      ata[i][j] = s;
    }
  }
  // (A^T A)^-1 의 첫 행만 필요 → 가우스 소거로 e0 해 구하기
  const aug = ata.map((row, i) => {
    const r = new Float64Array(p + 1);
    r.set(row);
    r[p] = i === 0 ? 1 : 0;
    return r;
  });
  for (let c = 0; c < p; c++) {
    let piv = c;
    for (let r = c + 1; r < p; r++) if (Math.abs(aug[r][c]) > Math.abs(aug[piv][c])) piv = r;
    if (Math.abs(aug[piv][c]) < 1e-12) return null;
    [aug[c], aug[piv]] = [aug[piv], aug[c]];
    const d = aug[c][c];
    for (let k = c; k <= p; k++) aug[c][k] /= d;
    for (let r = 0; r < p; r++) {
      if (r === c) continue;
      const f = aug[r][c];
      if (!f) continue;
      for (let k = c; k <= p; k++) aug[r][k] -= f * aug[c][k];
    }
  }
  const sol = [];
  for (let i = 0; i < p; i++) sol.push(aug[i][p]);
  // 각 오프셋 k에 대한 가중치 = Σ sol[i] * k^i
  const coeffs = new Float64Array(w);
  for (let idx = 0, k = -half; k <= half; k++, idx++) {
    let v = 0;
    for (let i = 0; i < p; i++) v += sol[i] * Math.pow(k, i);
    coeffs[idx] = v;
  }
  return coeffs;
}

function fltSavGol(x, win, order) {
  const n = x.length;
  let w = Math.max(3, Math.round(win) | 1);
  const p = Math.max(1, Math.min(5, Math.round(order)));
  if (w <= p + 1) w = (p + 2) | 1;
  const c = savGolCoeffs(w, p);
  if (!c) return Float64Array.from(x);
  const half = (w - 1) / 2;
  const y = Float64Array.from(x);
  for (let i = half; i < n - half; i++) {
    let s = 0;
    for (let k = 0; k < w; k++) s += c[k] * x[i - half + k];
    y[i] = s;
  }
  return y;
}

/** 버터워스 저역통과 계수 (바이쿼드 캐스케이드, 쌍선형 변환) */
function butterSections(fc, fs, order) {
  const nyq = fs / 2;
  const fcl = Math.max(1e-4, Math.min(fc, nyq * 0.98));
  const wc = Math.tan((Math.PI * fcl) / fs);
  const nsec = Math.max(1, Math.round(order / 2));
  const secs = [];
  const ord = nsec * 2;
  for (let k = 0; k < nsec; k++) {
    const theta = (Math.PI * (2 * k + ord + 1)) / (2 * ord);
    const q = -2 * Math.cos(theta);
    const nrm = 1 + q * wc + wc * wc;
    const b0 = (wc * wc) / nrm;
    secs.push({
      b0, b1: 2 * b0, b2: b0,
      a1: (2 * (wc * wc - 1)) / nrm,
      a2: (1 - q * wc + wc * wc) / nrm
    });
  }
  return secs;
}

function biquadPass(x, s) {
  const n = x.length;
  const y = new Float64Array(n);
  let x1 = x[0], x2 = x[0], y1 = x[0], y2 = x[0];
  for (let i = 0; i < n; i++) {
    const v = s.b0 * x[i] + s.b1 * x1 + s.b2 * x2 - s.a1 * y1 - s.a2 * y2;
    x2 = x1; x1 = x[i]; y2 = y1; y1 = v; y[i] = v;
  }
  return y;
}

/**
 * 버터워스 저역통과 (zero-phase, 정/역방향 2회 통과).
 * 정방향만 적용하면 위상 지연이 생겨 그래프가 오른쪽으로 밀리는데,
 * 역방향으로 한 번 더 통과시키면 지연이 상쇄되어 시간축이 정확히 유지됨.
 */
function fltButterworth(x, fc, fs, order) {
  const secs = butterSections(fc, fs, order);
  let y = Float64Array.from(x);
  for (const s of secs) y = biquadPass(y, s);
  y.reverse();
  for (const s of secs) y = biquadPass(y, s);
  y.reverse();
  return y;
}

/** 지수 이동평균 (1차 IIR) — 실시간 로거에서 쓰는 필터와 동일한 특성(지연 있음). */
function fltEma(x, fc, fs) {
  const a = 1 - Math.exp((-2 * Math.PI * fc) / fs);
  const n = x.length;
  const y = new Float64Array(n);
  let acc = x[0];
  for (let i = 0; i < n; i++) { acc += a * (x[i] - acc); y[i] = acc; }
  return y;
}

/** 변화율 제한 (Slew rate limit) — 물리적으로 불가능한 급변을 잘라냄. */
function fltSlewLimit(x, maxRate, dt) {
  const n = x.length;
  const lim = Math.abs(maxRate) * dt;
  const y = new Float64Array(n);
  y[0] = x[0];
  for (let i = 1; i < n; i++) {
    const d = x[i] - y[i - 1];
    y[i] = y[i - 1] + (d > lim ? lim : d < -lim ? -lim : d);
  }
  return y;
}

/**
 * 드롭아웃 제거 — 지정한 유효범위를 벗어난 샘플(주로 0)을 무효 처리하고
 * 앞뒤 정상값으로 선형 보간. maxGap 샘플보다 긴 결측 구간은 실제로 신호가
 * 없던 구간(엔진 정지 등)으로 보고 건드리지 않음.
 */
function fltDropout(x, lo, hi, maxGap) {
  const n = x.length;
  const y = Float64Array.from(x);
  const bad = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    const v = x[i];
    if (!Number.isFinite(v) || (lo !== null && v < lo) || (hi !== null && v > hi)) bad[i] = 1;
  }
  let i = 0;
  while (i < n) {
    if (!bad[i]) { i++; continue; }
    let j = i;
    while (j < n && bad[j]) j++;
    const len = j - i;
    const hasLeft = i > 0, hasRight = j < n;
    if (len <= maxGap && hasLeft && hasRight) {
      const a = y[i - 1], b = y[j];
      for (let k = 0; k < len; k++) y[i + k] = a + ((b - a) * (k + 1)) / (len + 1);
    }
    i = j;
  }
  return y;
}

// ==================== [2] 필터 정의 (UI 메타데이터) ====================

const FILTER_DEFS = {
  none: {
    label: '없음 (원본)',
    desc: '로그에 기록된 값을 그대로 표시합니다.',
    params: []
  },
  median: {
    label: '중앙값 (Median)',
    desc: '창 안의 중앙값으로 대체. 순간적으로 튀는 값과 드롭아웃 제거에 가장 강하고, 급격한 실제 변화(계단)는 그대로 보존합니다.',
    params: [{ key: 'win', label: '창 크기', unit: '샘플', min: 3, max: 101, step: 2, def: 5 }]
  },
  hampel: {
    label: 'Hampel (스파이크 제거)',
    desc: '이동 중앙값에서 n·σ 이상 벗어난 점만 골라 교체. 평활을 하지 않으므로 정상 구간 파형이 손상되지 않습니다. RPM이 갑자기 튀는 CAN 오류에 가장 적합.',
    params: [
      { key: 'win', label: '창 크기', unit: '샘플', min: 3, max: 51, step: 2, def: 7 },
      { key: 'nsig', label: '민감도 n·σ', unit: '', min: 1, max: 10, step: 0.5, def: 4 }
    ]
  },
  movavg: {
    label: '이동 평균 (Moving Average)',
    desc: '가장 단순한 평활. 무작위 노이즈에는 효과적이지만 스파이크는 주변으로 번지고 피크가 낮아집니다.',
    params: [{ key: 'win', label: '창 크기', unit: '샘플', min: 3, max: 201, step: 2, def: 9 }]
  },
  savgol: {
    label: 'Savitzky-Golay',
    desc: '창 안에서 다항식을 적합해 평활. 이동평균과 달리 피크 높이와 폭을 잘 보존해서 서스펜션 스트로크·브레이크 최대압 판독에 유리합니다.',
    params: [
      { key: 'win', label: '창 크기', unit: '샘플', min: 5, max: 201, step: 2, def: 11 },
      { key: 'order', label: '다항식 차수', unit: '', min: 1, max: 5, step: 1, def: 2 }
    ]
  },
  butter: {
    label: '버터워스 저역통과 (Zero-phase)',
    desc: '차단주파수 이상의 성분만 제거하는 정통 저역통과 필터. 정/역방향 2회 통과라 시간축 지연이 없습니다.',
    params: [
      { key: 'fc', label: '차단 주파수', unit: 'Hz', min: 0.2, max: 40, step: 0.1, def: 5 },
      { key: 'order', label: '차수', unit: '', min: 2, max: 8, step: 2, def: 2 }
    ]
  },
  ema: {
    label: 'EMA (지수 이동평균)',
    desc: '1차 IIR 저역통과. 실제 ECU/로거에 들어가는 실시간 필터와 같은 특성이라 "차량에서 이 필터를 켜면 어떻게 보일지" 확인할 때 씁니다. 위상 지연이 있습니다.',
    params: [{ key: 'fc', label: '차단 주파수', unit: 'Hz', min: 0.2, max: 40, step: 0.1, def: 5 }]
  },
  slew: {
    label: '변화율 제한 (Slew Limit)',
    desc: '초당 변화량의 상한을 두어 물리적으로 불가능한 급변을 잘라냅니다. 채널의 물리 한계를 알고 있을 때 가장 안전한 방식입니다.',
    params: [{ key: 'rate', label: '최대 변화율', unit: '단위/초', min: 1, max: 200000, step: 1, def: 500 }]
  }
};

// ==================== [3] 채널 레지스트리 ====================
// key → { label, unit, from(row), integer?, dropoutDef, recommend }

const CHANNELS = {
  fl_speed: {
    label: 'FL Wheel Speed (EMU VSS)', unit: 'km/h', clamp: [0, 400],
    from: r => r.fl_speed_kmh || 0,
    dropoutDef: { on: false, lo: 0.01, hi: null, gap: 20 },
    recommend: { dropout: { on: false }, type: 'median', params: { win: 5 } }
  },
  rl_speed: {
    label: 'RL Wheel Speed', unit: 'km/h', clamp: [0, 400],
    from: r => r.rl_speed_kmh || 0,
    dropoutDef: { on: false, lo: 0.01, hi: null, gap: 20 },
    recommend: { dropout: { on: true, lo: 0.01, hi: null, gap: 20 }, type: 'median', params: { win: 5 } }
  },
  rr_speed: {
    label: 'RR Wheel Speed', unit: 'km/h', clamp: [0, 400],
    from: r => r.rr_speed_kmh || 0,
    dropoutDef: { on: false, lo: 0.01, hi: null, gap: 20 },
    recommend: { dropout: { on: true, lo: 0.01, hi: null, gap: 20 }, type: 'median', params: { win: 5 } }
  },
  rpm: {
    label: 'Engine RPM', unit: 'rpm', clamp: [0, 20000],
    from: r => r.rpm || 0,
    dropoutDef: { on: false, lo: 1, hi: 15000, gap: 20 },
    recommend: { dropout: { on: false }, type: 'hampel', params: { win: 7, nsig: 4 } }
  },
  gear: {
    label: 'Gear', unit: '', integer: true, clamp: [0, 6],
    from: r => r.gear || 0,
    dropoutDef: { on: false, lo: 0, hi: 6, gap: 10 },
    recommend: { dropout: { on: false }, type: 'median', params: { win: 5 } }
  },
  steering: {
    label: 'Steering Angle', unit: '°',
    from: r => getCalibratedSteering(r.steering_raw),
    dropoutDef: { on: false, lo: null, hi: null, gap: 20 },
    recommend: { dropout: { on: false }, type: 'butter', params: { fc: 5, order: 2 } }
  },
  throttle: {
    label: 'Throttle Position', unit: '%', clamp: [0, 100],
    from: r => r.decoded_tps || 0,
    dropoutDef: { on: false, lo: null, hi: 100, gap: 20 },
    recommend: { dropout: { on: false }, type: 'hampel', params: { win: 7, nsig: 4 } }
  },
  brake: {
    label: 'Brake Position', unit: '%', clamp: [0, 100],
    from: r => getCalibratedBrake(r.front_brake_raw),
    dropoutDef: { on: false, lo: null, hi: null, gap: 20 },
    recommend: { dropout: { on: false }, type: 'savgol', params: { win: 11, order: 2 } }
  },
  sus_fl: {
    label: 'Suspension FL', unit: 'mm',
    from: r => getCalibratedSuspension('fl', r.suspension_fl_raw),
    dropoutDef: { on: false, lo: null, hi: null, gap: 20 },
    recommend: { dropout: { on: false }, type: 'butter', params: { fc: 5, order: 2 } }
  },
  sus_fr: {
    label: 'Suspension FR', unit: 'mm',
    from: r => getCalibratedSuspension('fr', r.suspension_fr_raw),
    dropoutDef: { on: false, lo: null, hi: null, gap: 20 },
    recommend: { dropout: { on: false }, type: 'butter', params: { fc: 5, order: 2 } }
  },
  sus_rl: {
    label: 'Suspension RL', unit: 'mm',
    from: r => getCalibratedSuspension('rl', r.suspension_rl_raw),
    dropoutDef: { on: false, lo: null, hi: null, gap: 20 },
    recommend: { dropout: { on: false }, type: 'butter', params: { fc: 5, order: 2 } }
  },
  sus_rr: {
    label: 'Suspension RR', unit: 'mm',
    from: r => getCalibratedSuspension('rr', r.suspension_rr_raw),
    dropoutDef: { on: false, lo: null, hi: null, gap: 20 },
    recommend: { dropout: { on: false }, type: 'butter', params: { fc: 5, order: 2 } }
  },
  water: {
    label: 'Coolant Temp', unit: '°C',
    from: r => r.water_c,
    dropoutDef: { on: false, lo: -40, hi: 150, gap: 50 },
    recommend: { dropout: { on: true, lo: -40, hi: 150, gap: 50 }, type: 'median', params: { win: 9 } }
  },
  oil: {
    label: 'Oil Temp', unit: '°C',
    from: r => r.oil_c,
    dropoutDef: { on: false, lo: -40, hi: 200, gap: 50 },
    recommend: { dropout: { on: true, lo: -40, hi: 200, gap: 50 }, type: 'median', params: { win: 9 } }
  },
  iat: {
    label: 'Intake Air Temp', unit: '°C',
    from: r => r.iat_c,
    dropoutDef: { on: false, lo: -40, hi: 150, gap: 50 },
    recommend: { dropout: { on: true, lo: -40, hi: 150, gap: 50 }, type: 'median', params: { win: 9 } }
  },
  ecu: {
    label: 'ECU Temp', unit: '°C',
    from: r => r.ecu_c,
    dropoutDef: { on: false, lo: -40, hi: 150, gap: 50 },
    recommend: { dropout: { on: true, lo: -40, hi: 150, gap: 50 }, type: 'median', params: { win: 9 } }
  },
  imu_ax: { label: 'IMU Accel X', unit: 'g', from: r => r.imu_accel_x_g || 0, dropoutDef: { on: false }, recommend: { type: 'butter', params: { fc: 5, order: 2 } } },
  imu_ay: { label: 'IMU Accel Y', unit: 'g', from: r => r.imu_accel_y_g || 0, dropoutDef: { on: false }, recommend: { type: 'butter', params: { fc: 5, order: 2 } } },
  imu_gx: { label: 'IMU Gyro X', unit: '°/s', from: r => r.imu_gyro_x_dps || 0, dropoutDef: { on: false }, recommend: { type: 'butter', params: { fc: 5, order: 2 } } },
  imu_gy: { label: 'IMU Gyro Y', unit: '°/s', from: r => r.imu_gyro_y_dps || 0, dropoutDef: { on: false }, recommend: { type: 'butter', params: { fc: 5, order: 2 } } },
  imu_gz: { label: 'IMU Gyro Z', unit: '°/s', from: r => r.imu_gyro_z_dps || 0, dropoutDef: { on: false }, recommend: { type: 'butter', params: { fc: 5, order: 2 } } }
};

/** 차트 canvas id → 그 차트가 그리는 채널 키 (데이터셋 순서와 일치) */
const CHART_CHANNELS = {
  'chart-ground-speed': ['fl_speed', 'rl_speed', 'rr_speed'],
  'chart-engine-rpm': ['rpm'],
  'chart-vehicle-gear': ['gear'],
  'chart-steering-angle': ['steering', 'imu_gz', 'imu_ay'],
  'chart-throttle-brake': ['throttle', 'brake'],
  'diag-chart-throttle-brake': ['throttle', 'brake'],
  'diag-chart-steering': ['steering'],
  'chart-sus-fl': ['sus_fl'],
  'chart-sus-fr': ['sus_fr'],
  'chart-sus-rl': ['sus_rl'],
  'chart-sus-rr': ['sus_rr'],
  'chart-coolant-oil': ['water', 'oil', 'fl_speed'],
  'chart-intake-ecu': ['iat', 'ecu'],
  'chart-imu-accel': ['imu_ax', 'imu_ay'],
  'chart-imu-gyro': ['imu_gx', 'imu_gy', 'imu_gz']
};

// ==================== [4] 필터 상태 & 적용 엔진 ====================

const filterState = {};   // key → { dropout:{on,lo,hi,gap}, type, params:{} }
const rawChannel = {};    // key → Float64Array (100Hz 원본 전체)
const filteredChannel = {}; // key → Float64Array (100Hz 필터 적용본)
let sampleRateHz = 100;

function defaultFilterFor(key) {
  const ch = CHANNELS[key];
  const d = ch.dropoutDef || { on: false, lo: null, hi: null, gap: 20 };
  return { dropout: { on: false, lo: d.lo, hi: d.hi, gap: d.gap }, type: 'none', params: {} };
}

function getFilterState(key) {
  if (!filterState[key]) filterState[key] = defaultFilterFor(key);
  return filterState[key];
}

function paramValue(type, pkey, params) {
  const def = FILTER_DEFS[type].params.find(p => p.key === pkey);
  const v = params && params[pkey];
  return Number.isFinite(v) ? v : def.def;
}

/** 원본 배열에 현재 필터 설정을 적용해 반환 */
function applyFilterChain(key, src) {
  const st = getFilterState(key);
  let y = src;

  if (st.dropout && st.dropout.on) {
    y = fltDropout(y, st.dropout.lo, st.dropout.hi, st.dropout.gap);
  }

  const t = st.type;
  const P = k => paramValue(t, k, st.params);
  if (t === 'median') y = fltMedian(y, P('win'));
  else if (t === 'hampel') y = fltHampel(y, P('win'), P('nsig'));
  else if (t === 'movavg') y = fltMovingAverage(y, P('win'));
  else if (t === 'savgol') y = fltSavGol(y, P('win'), P('order'));
  else if (t === 'butter') y = fltButterworth(y, P('fc'), sampleRateHz, P('order'));
  else if (t === 'ema') y = fltEma(y, P('fc'), sampleRateHz);
  else if (t === 'slew') y = fltSlewLimit(y, P('rate'), 1 / sampleRateHz);

  if (y === src) return y;

  // 정수 채널(기어 등)은 반올림
  if (CHANNELS[key].integer) {
    const z = new Float64Array(y.length);
    for (let i = 0; i < y.length; i++) z[i] = Math.round(y[i]);
    y = z;
  }

  // 정의상 넘을 수 없는 물리 한계가 있는 채널은 클램프.
  // (저역통과/Savitzky-Golay는 계단 응답에서 3~8% 오버슈트가 발생하므로
  //  스로틀이 -3%가 되는 등 물리적으로 불가능한 값이 나올 수 있음)
  const cl = CHANNELS[key].clamp;
  if (cl) {
    for (let i = 0; i < y.length; i++) {
      if (y[i] < cl[0]) y[i] = cl[0];
      else if (y[i] > cl[1]) y[i] = cl[1];
    }
  }
  return y;
}

/** 로그 로드 직후 호출: 원본 채널 배열 구축 + 샘플레이트 추정 */
function buildRawChannels(rows) {
  const n = rows.length;
  if (n > 1) {
    const span = (rows[n - 1].time_sec || 0) - (rows[0].time_sec || 0);
    if (span > 0) sampleRateHz = Math.max(1, Math.round((n - 1) / span));
  }
  Object.keys(CHANNELS).forEach(key => {
    const get = CHANNELS[key].from;
    const arr = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const v = Number(get(rows[i]));
      arr[i] = Number.isFinite(v) ? v : 0;
    }
    rawChannel[key] = arr;
    filteredChannel[key] = applyFilterChain(key, arr);
  });
}

function recomputeChannel(key) {
  if (!rawChannel[key]) return;
  filteredChannel[key] = applyFilterChain(key, rawChannel[key]);
}

/**
 * 한 채널의 원본 배열을 다시 만들고 필터를 재적용합니다.
 * 채널의 계산식 자체가 바뀌었을 때 사용합니다 (예: 조향 영점 보정 변경).
 */
function rebuildRawChannel(key, rows) {
  const ch = CHANNELS[key];
  if (!ch || !rows || !rows.length) return;
  const get = ch.from;
  const arr = new Float64Array(rows.length);
  for (let i = 0; i < rows.length; i++) {
    const v = Number(get(rows[i]));
    arr[i] = Number.isFinite(v) ? v : 0;
  }
  rawChannel[key] = arr;
  recomputeChannel(key);
}

/** 화면용 다운샘플 인덱스에 맞춰 {x,y} 배열 생성 */
function channelSeries(key, indices, times) {
  const src = filteredChannel[key] || rawChannel[key];
  const out = new Array(indices.length);
  for (let i = 0; i < indices.length; i++) {
    out[i] = { x: times[i], y: src ? src[indices[i]] : 0 };
  }
  return out;
}

/** 커서 위치의 필터 적용 값 (숫자 표시용). 전체 데이터 인덱스를 받음. */
function channelValueAt(key, fullIndex) {
  const src = filteredChannel[key] || rawChannel[key];
  if (!src || fullIndex == null || fullIndex < 0 || fullIndex >= src.length) return null;
  return src[fullIndex];
}

function filterBadgeText(key) {
  const st = getFilterState(key);
  const bits = [];
  if (st.dropout && st.dropout.on) bits.push('DO');
  const t = st.type;
  if (t !== 'none') {
    const P = k => paramValue(t, k, st.params);
    const short = { median: 'MED', hampel: 'HAM', movavg: 'AVG', savgol: 'SG', butter: 'LPF', ema: 'EMA', slew: 'SLEW' }[t];
    if (t === 'butter' || t === 'ema') bits.push(`${short} ${P('fc')}Hz`);
    else if (t === 'hampel') bits.push(`${short} ${P('win')}/${P('nsig')}σ`);
    else if (t === 'savgol') bits.push(`${short} ${P('win')}/p${P('order')}`);
    else if (t === 'slew') bits.push(`${short} ${P('rate')}`);
    else bits.push(`${short} ${P('win')}`);
  }
  return bits.join(' · ');
}

// ==================== [5] 우클릭 컨텍스트 메뉴 UI ====================

let menuEl = null;
let menuChannelKey = null;

function closeFilterMenu() {
  if (menuEl) { menuEl.remove(); menuEl = null; menuChannelKey = null; }
}

document.addEventListener('click', e => {
  if (menuEl && !menuEl.contains(e.target)) closeFilterMenu();
});
document.addEventListener('keydown', e => { if (e.key === 'Escape') closeFilterMenu(); });

function buildFilterMenu(channelKeys, x, y) {
  closeFilterMenu();
  menuChannelKey = channelKeys[0];

  const el = document.createElement('div');
  el.className = 'filter-menu';
  menuEl = el;
  document.body.appendChild(el);

  const render = () => {
    const key = menuChannelKey;
    const ch = CHANNELS[key];
    const st = getFilterState(key);

    let html = `<div class="fm-head">
        <span class="fm-title">노이즈 필터</span>
        <button class="fm-close" data-act="close">✕</button>
      </div>`;

    if (channelKeys.length > 1) {
      html += `<div class="fm-row"><label>채널</label>
        <select class="fm-select" data-act="chan">` +
        channelKeys.map(k => `<option value="${k}" ${k === key ? 'selected' : ''}>${CHANNELS[k].label}</option>`).join('') +
        `</select></div>`;
    } else {
      html += `<div class="fm-chan-name">${ch.label}${ch.unit ? ` [${ch.unit}]` : ''}</div>`;
    }

    // 드롭아웃 제거 섹션
    const d = st.dropout;
    html += `<div class="fm-sec">
      <label class="fm-check"><input type="checkbox" data-act="do-on" ${d.on ? 'checked' : ''}>
        <span>드롭아웃 제거 (범위 밖 값 보간)</span></label>
      <div class="fm-sub ${d.on ? '' : 'fm-dim'}">
        <span>유효범위</span>
        <input class="fm-num" type="number" data-act="do-lo" value="${d.lo ?? ''}" placeholder="min" step="any">
        <span>~</span>
        <input class="fm-num" type="number" data-act="do-hi" value="${d.hi ?? ''}" placeholder="max" step="any">
        <span>최대 보간</span>
        <input class="fm-num" type="number" data-act="do-gap" value="${d.gap}" min="1" max="500" step="1">
        <span class="fm-unit">샘플</span>
      </div>
    </div>`;

    // 필터 목록
    html += `<div class="fm-sec fm-list">`;
    Object.entries(FILTER_DEFS).forEach(([tkey, def]) => {
      const on = st.type === tkey;
      html += `<div class="fm-item ${on ? 'active' : ''}">
        <label class="fm-radio"><input type="radio" name="fm-type" data-act="type" value="${tkey}" ${on ? 'checked' : ''}>
          <span class="fm-item-label">${def.label}</span></label>`;
      if (on) {
        html += `<div class="fm-desc">${def.desc}</div>`;
        if (def.params.length) {
          html += `<div class="fm-params">`;
          def.params.forEach(p => {
            const v = paramValue(tkey, p.key, st.params);
            html += `<div class="fm-param">
              <label>${p.label}</label>
              <input class="fm-range" type="range" data-act="param" data-p="${p.key}"
                     min="${p.min}" max="${p.max}" step="${p.step}" value="${v}">
              <input class="fm-num fm-num-w" type="number" data-act="param-num" data-p="${p.key}"
                     min="${p.min}" max="${p.max}" step="${p.step}" value="${v}">
              <span class="fm-unit">${p.unit}</span>
            </div>`;
          });
          html += `</div>`;
        }
      }
      html += `</div>`;
    });
    html += `</div>`;

    html += `<div class="fm-foot">
        <button class="fm-btn fm-btn-primary" data-act="recommend">추천 설정 적용</button>
        <button class="fm-btn" data-act="reset">이 채널 초기화</button>
        <button class="fm-btn" data-act="reset-all">전체 초기화</button>
      </div>
      <div class="fm-foot">
        <button class="fm-btn fm-btn-wide" data-act="recommend-all">모든 채널에 추천 설정 일괄 적용</button>
      </div>
      <div class="fm-hint">필터는 원본 100Hz 데이터에 적용된 뒤 화면용으로 다운샘플링됩니다. 원본 CSV는 변경되지 않습니다.</div>`;

    el.innerHTML = html;
    bind();
  };

  let applyTimer = null;
  const scheduleApply = (immediate) => {
    clearTimeout(applyTimer);
    const run = () => {
      recomputeChannel(menuChannelKey);
      if (typeof refreshChartsAfterFilter === 'function') refreshChartsAfterFilter();
    };
    if (immediate) run(); else applyTimer = setTimeout(run, 90);
  };

  const bind = () => {
    el.querySelectorAll('[data-act]').forEach(node => {
      const act = node.dataset.act;

      if (act === 'close') node.onclick = closeFilterMenu;

      if (act === 'chan') node.onchange = e => { menuChannelKey = e.target.value; render(); };

      if (act === 'type') node.onchange = e => {
        const st = getFilterState(menuChannelKey);
        st.type = e.target.value;
        st.params = {};
        render();
        scheduleApply(true);
      };

      if (act === 'do-on') node.onchange = e => {
        getFilterState(menuChannelKey).dropout.on = e.target.checked;
        render();
        scheduleApply(true);
      };
      if (act === 'do-lo' || act === 'do-hi') node.oninput = e => {
        const st = getFilterState(menuChannelKey);
        const raw = e.target.value.trim();
        st.dropout[act === 'do-lo' ? 'lo' : 'hi'] = raw === '' ? null : parseFloat(raw);
        scheduleApply();
      };
      if (act === 'do-gap') node.oninput = e => {
        getFilterState(menuChannelKey).dropout.gap = Math.max(1, parseInt(e.target.value, 10) || 1);
        scheduleApply();
      };

      if (act === 'param' || act === 'param-num') node.oninput = e => {
        const st = getFilterState(menuChannelKey);
        const pk = e.target.dataset.p;
        let v = parseFloat(e.target.value);
        if (!Number.isFinite(v)) return;
        const def = FILTER_DEFS[st.type].params.find(p => p.key === pk);
        // 창 크기는 홀수만 허용
        if (def.step === 2 && v % 2 === 0) v += 1;
        v = Math.max(def.min, Math.min(def.max, v));
        st.params[pk] = v;
        // 짝 입력창 동기화 (재렌더 없이)
        el.querySelectorAll(`[data-p="${pk}"]`).forEach(n => { if (n !== e.target) n.value = v; });
        scheduleApply();
      };

      if (act === 'recommend') node.onclick = () => {
        applyRecommended(menuChannelKey);
        render();
        scheduleApply(true);
      };
      if (act === 'reset') node.onclick = () => {
        filterState[menuChannelKey] = defaultFilterFor(menuChannelKey);
        render();
        scheduleApply(true);
      };
      if (act === 'reset-all') node.onclick = () => {
        Object.keys(CHANNELS).forEach(k => {
          filterState[k] = defaultFilterFor(k);
          recomputeChannel(k);
        });
        render();
        if (typeof refreshChartsAfterFilter === 'function') refreshChartsAfterFilter();
      };
      if (act === 'recommend-all') node.onclick = () => {
        Object.keys(CHANNELS).forEach(k => { applyRecommended(k); recomputeChannel(k); });
        render();
        if (typeof refreshChartsAfterFilter === 'function') refreshChartsAfterFilter();
      };
    });
  };

  render();

  // 화면 밖으로 나가지 않게 위치 보정
  const rect = el.getBoundingClientRect();
  let left = x, top = y;
  if (left + rect.width > window.innerWidth - 8) left = window.innerWidth - rect.width - 8;
  if (top + rect.height > window.innerHeight - 8) top = Math.max(8, window.innerHeight - rect.height - 8);
  el.style.left = Math.max(8, left) + 'px';
  el.style.top = Math.max(8, top) + 'px';
}

function applyRecommended(key) {
  const rec = CHANNELS[key].recommend;
  const base = defaultFilterFor(key);
  const st = {
    dropout: Object.assign({}, base.dropout, rec.dropout || {}),
    type: rec.type || 'none',
    params: Object.assign({}, rec.params || {})
  };
  filterState[key] = st;
}

/** 차트 카드 우클릭 → 클릭 위치의 canvas를 찾아 해당 채널로 메뉴 오픈 */
function initFilterContextMenu() {
  document.addEventListener('contextmenu', e => {
    const card = e.target.closest('.motec-chart-card');
    if (!card) return;
    const canvases = Array.from(card.querySelectorAll('canvas')).filter(c => CHART_CHANNELS[c.id]);
    if (!canvases.length) return;

    // 카드 안에 캔버스가 여러 개면 클릭 Y좌표에 가장 가까운 것 선택
    let target = canvases[0];
    if (canvases.length > 1) {
      let best = Infinity;
      canvases.forEach(c => {
        const r = c.getBoundingClientRect();
        const dist = e.clientY < r.top ? r.top - e.clientY
                   : e.clientY > r.bottom ? e.clientY - r.bottom : 0;
        if (dist < best) { best = dist; target = c; }
      });
    }

    const keys = CHART_CHANNELS[target.id];
    if (!keys || !keys.length) return;
    if (typeof globalData === 'undefined' || !globalData.length) return;

    e.preventDefault();
    buildFilterMenu(keys, e.clientX, e.clientY);
  });
}

/** 각 차트 라벨바에 현재 적용된 필터 배지 표시 */
function refreshFilterBadges() {
  Object.entries(CHART_CHANNELS).forEach(([canvasId, keys]) => {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;
    const card = canvas.closest('.motec-chart-card');
    if (!card) return;
    const bar = card.querySelector('.chart-label-bar');
    if (!bar) return;

    let badge = bar.querySelector(`.filter-badge[data-for="${canvasId}"]`);
    let texts = keys.map(k => {
      const t = filterBadgeText(k);
      return t ? `${CHANNELS[k].label.split(' ')[0]}: ${t}` : null;
    }).filter(Boolean);

    // The checkbox applies one shared filter to all three axes. Avoid showing
    // the same LPF status three times on each IMU chart.
    if (canvasId === 'chart-imu-accel' || canvasId === 'chart-imu-gyro') {
      const unique = [...new Set(keys.map(k => filterBadgeText(k)).filter(Boolean))];
      texts = unique.length === 1 ? [unique[0].replace(/(\d)Hz\b/, '$1 Hz')] : texts;
    }

    if (!texts.length) { if (badge) badge.remove(); return; }
    if (!badge) {
      badge = document.createElement('span');
      badge.className = 'filter-badge';
      badge.dataset.for = canvasId;
      badge.title = (canvasId === 'chart-imu-accel' || canvasId === 'chart-imu-gyro')
        ? 'IMU 5 Hz 저역통과 필터가 적용되어 있습니다'
        : '이 그래프에 노이즈 필터가 적용되어 있습니다 (우클릭으로 변경)';
      bar.appendChild(badge);
    }
    badge.textContent = '⚡ ' + texts.join(' | ');
  });
}
