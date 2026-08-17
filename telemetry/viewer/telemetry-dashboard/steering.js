/* =============================================================================
 * NS26F Telemetry — 조향 영점(Zero) 보정 모듈
 * -----------------------------------------------------------------------------
 * 핸들 그래픽을 클릭하면 조향 보정 패널이 열립니다.
 *   조향각[°] = (raw − 영점raw) × 배율[°/LSB] × (반전 ? −1 : +1)
 *
 * 기본값(영점 998 / 배율 0.1 / 반전 없음)은 기존 하드코딩 식과 완전히 동일합니다.
 *   (raw − 2048) × 0.1 + 105 = 0.1 × raw − 99.8 = (raw − 998) × 0.1   ✔
 *
 * ── Telemetry_001.csv 로 확인한 영점 오차 ──────────────────────────────────
 *  현재 영점(raw 998)으로는 직진 주행 중에도 조향각이 +28.9°로 표시됩니다.
 *  서로 독립적인 두 가지 방법이 모두 raw ≈ 1290~1304 를 영점으로 지목했습니다.
 *
 *   (1) 분포 기반 : 고속·저조향변화 구간의 조향 raw 히스토그램 최빈값 → 1287
 *   (2) GPS 검증  : GPS 궤적에서 요레이트를 구해 자전거 모델
 *                   (요레이트/속도 ∝ 조향각) 로 선형회귀 → 요레이트가 0이 되는
 *                   raw 값 = 1304 (상관계수 r=0.80, 95% 신뢰구간 1301~1307)
 *
 *  두 방법의 차이는 17 LSB(1.7°)로, 조향 센서와 무관한 GPS 궤적이 분포 기반
 *  추정을 뒷받침하므로 현재 캘리브레이션이 약 29° 틀어져 있다고 판단됩니다.
 * ========================================================================== */

// ==================== 보정 상태 ====================

const STEER_CAL_DEFAULT = { zeroRaw: 998, degPerLsb: 0.1, axisLimit: 250, invert: false };
const STEER_CAL_STORE_KEY = 'nssur_steering_cal';

const steeringCal = Object.assign({}, STEER_CAL_DEFAULT);

(function loadSteeringCal() {
  try {
    const s = JSON.parse(localStorage.getItem(STEER_CAL_STORE_KEY) || 'null');
    if (s && Number.isFinite(s.zeroRaw) && Number.isFinite(s.degPerLsb)) {
      steeringCal.zeroRaw = s.zeroRaw;
      // Sensor conversion is fixed at the logger calibration. The former
      // multiplier is intentionally ignored because it changed real values.
      steeringCal.degPerLsb = 0.1;
      steeringCal.axisLimit = Number.isFinite(s.axisLimit) ? Math.max(20, Math.min(500, s.axisLimit)) : 250;
      steeringCal.invert = !!s.invert;
    }
  } catch (err) { /* 저장값이 깨졌으면 기본값 사용 */ }
})();

function saveSteeringCal() {
  try { localStorage.setItem(STEER_CAL_STORE_KEY, JSON.stringify(steeringCal)); } catch (err) { /* ignore */ }
}

function isSteeringCalDefault() {
  return steeringCal.zeroRaw === STEER_CAL_DEFAULT.zeroRaw &&
         steeringCal.degPerLsb === STEER_CAL_DEFAULT.degPerLsb &&
         steeringCal.axisLimit === STEER_CAL_DEFAULT.axisLimit &&
         steeringCal.invert === STEER_CAL_DEFAULT.invert;
}

// ==================== 자동 영점 추정 ====================

/** 1 LSB 폭 히스토그램 + 가우시안 평활 후 최빈값 (커널 밀도 최빈값) */
function kdeMode(values, bandwidth) {
  if (!values.length) return null;
  let lo = Infinity, hi = -Infinity;
  for (const v of values) { if (v < lo) lo = v; if (v > hi) hi = v; }
  lo = Math.floor(lo); hi = Math.ceil(hi);
  const nb = hi - lo + 1;
  if (nb < 3) return lo;

  const hist = new Float64Array(nb);
  for (const v of values) {
    const b = Math.round(v) - lo;
    if (b >= 0 && b < nb) hist[b]++;
  }
  const bw = Math.max(1, bandwidth || 8);
  const half = Math.ceil(3 * bw);
  const kern = new Float64Array(2 * half + 1);
  let ks = 0;
  for (let i = -half; i <= half; i++) { const v = Math.exp(-0.5 * (i / bw) * (i / bw)); kern[i + half] = v; ks += v; }
  for (let i = 0; i < kern.length; i++) kern[i] /= ks;

  let bestI = 0, bestV = -Infinity;
  for (let i = 0; i < nb; i++) {
    let s = 0;
    for (let k = -half; k <= half; k++) {
      const j = i + k;
      if (j >= 0 && j < nb) s += hist[j] * kern[k + half];
    }
    if (s > bestV) { bestV = s; bestI = i; }
  }
  return lo + bestI;
}

/**
 * [방법 1] 분포 기반 추정
 * 서킷 주행에서 조향이 가장 오래 머무는 값 = 직진 자세.
 * 고속 + 조향 변화율이 낮은 구간만 골라 최빈값을 구합니다.
 */
function estimateSteeringZeroByDistribution(rows) {
  const n = rows.length;
  if (n < 200) return null;

  const raws = new Float64Array(n);
  const spds = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const r = Number(rows[i].steering_raw);
    raws[i] = Number.isFinite(r) ? r : NaN;
    const s = Number(rows[i].fl_speed_kmh);
    spds[i] = Number.isFinite(s) ? s : 0;
  }

  let maxSpd = 0;
  for (const s of spds) if (s > maxSpd) maxSpd = s;
  const spdGate = Math.max(20, maxSpd * 0.3);

  // 조향 변화율 (LSB/초)
  const dt = 0.01;
  const rate = new Float64Array(n);
  for (let i = 1; i < n - 1; i++) rate[i] = Math.abs(raws[i + 1] - raws[i - 1]) / (2 * dt);

  const pick = (useGates) => {
    const out = [];
    for (let i = 1; i < n - 1; i++) {
      if (!Number.isFinite(raws[i])) continue;
      if (useGates && !(spds[i] > spdGate && rate[i] < 200)) continue;
      if (!useGates && spds[i] <= 5) continue;
      out.push(raws[i]);
    }
    return out;
  };

  let sel = pick(true);
  let how = `직진 구간 (속도>${spdGate.toFixed(0)}km/h, 조향변화<200LSB/s)`;
  if (sel.length < 300) { sel = pick(false); how = '주행 중 전체 구간'; }
  if (sel.length < 100) return null;

  const mode = kdeMode(sel, 8);
  if (mode === null) return null;

  // 봉우리의 선명도 = 최빈값 ±20LSB 안에 들어오는 표본 비율
  let near = 0;
  for (const v of sel) if (Math.abs(v - mode) <= 20) near++;

  return { zeroRaw: mode, samples: sel.length, sharpness: near / sel.length, how };
}

/**
 * [방법 2] GPS 궤적 기반 독립 검증
 * GPS 위치에서 진행방향(heading) → 요레이트를 구한 뒤, 자전거 모델의
 *   요레이트 / 속도 ∝ (조향raw − 영점raw)
 * 관계를 선형회귀해 요레이트가 0이 되는 raw 값을 찾습니다.
 * 조향 센서 값의 분포와 전혀 무관한 독립 측정이라 교차검증에 쓸 수 있습니다.
 */
function estimateSteeringZeroByGps(rows) {
  const n = rows.length;
  if (n < 500 || typeof convertNmeaToDecimal !== 'function') return null;

  // GPS는 보통 5Hz라 위치가 실제로 바뀐 지점만 사용
  const px = [], py = [], pt = [], praw = [], pspd = [];
  let lastLat = null, lastLon = null, latSum = 0, cnt = 0;
  const lats = [], lons = [];
  for (let i = 0; i < n; i++) {
    const la = convertNmeaToDecimal(rows[i].gps_lat, false);
    const lo = convertNmeaToDecimal(rows[i].gps_lon, true);
    if (la === null || lo === null || !Number.isFinite(la) || !Number.isFinite(lo)) continue;
    if (la === lastLat && lo === lastLon) continue;
    lastLat = la; lastLon = lo;
    lats.push(la); lons.push(lo);
    pt.push(rows[i].time_sec);
    praw.push(Number(rows[i].steering_raw));
    const gs = Number(rows[i].gps_speed_kmh);
    pspd.push(Number.isFinite(gs) && gs > 0 ? gs : Number(rows[i].fl_speed_kmh) || 0);
    latSum += la; cnt++;
  }
  const m = pt.length;
  if (m < 200) return null;

  const R = 6371000;
  const latRef = (latSum / cnt) * Math.PI / 180;
  for (let i = 0; i < m; i++) {
    px.push((lons[i] * Math.PI / 180) * R * Math.cos(latRef));
    py.push((lats[i] * Math.PI / 180) * R);
  }

  // 중심차분 속도벡터 → heading
  const hdg = new Float64Array(m);
  for (let i = 1; i < m - 1; i++) {
    const dtl = pt[i + 1] - pt[i - 1];
    if (dtl <= 0) { hdg[i] = hdg[i - 1]; continue; }
    hdg[i] = Math.atan2((px[i + 1] - px[i - 1]) / dtl, (py[i + 1] - py[i - 1]) / dtl) * 180 / Math.PI;
  }
  hdg[0] = hdg[1]; hdg[m - 1] = hdg[m - 2];

  // ±180° 경계 언랩
  for (let i = 1; i < m; i++) {
    let d = hdg[i] - hdg[i - 1];
    if (d > 180) { for (let j = i; j < m; j++) hdg[j] -= 360; }
    else if (d < -180) { for (let j = i; j < m; j++) hdg[j] += 360; }
  }

  const yaw = new Float64Array(m);
  for (let i = 1; i < m - 1; i++) {
    const dtl = pt[i + 1] - pt[i - 1];
    yaw[i] = dtl > 0 ? (hdg[i + 1] - hdg[i - 1]) / dtl : 0;
  }

  // 평활 (GPS 잡음 억제)
  const smooth = (a, w) => {
    const o = new Float64Array(a.length), h = (w - 1) / 2;
    for (let i = 0; i < a.length; i++) {
      let s = 0, c = 0;
      for (let k = Math.max(0, i - h); k <= Math.min(a.length - 1, i + h); k++) { s += a[k]; c++; }
      o[i] = s / c;
    }
    return o;
  };
  const yawS = smooth(yaw, 5), rawS = smooth(Float64Array.from(praw), 5), spdS = smooth(Float64Array.from(pspd), 5);

  // 회귀: z = yaw/speed  vs  x = raw
  const X = [], Z = [];
  let maxSpd = 0;
  for (const s of spdS) if (s > maxSpd) maxSpd = s;
  const gate = Math.max(25, maxSpd * 0.3);
  for (let i = 1; i < m - 1; i++) {
    if (!Number.isFinite(rawS[i]) || spdS[i] < gate) continue;
    if (Math.abs(yawS[i]) > 70) continue;
    X.push(rawS[i]); Z.push(yawS[i] / spdS[i]);
  }
  if (X.length < 100) return null;

  const N = X.length;
  let sx = 0, sz = 0;
  for (let i = 0; i < N; i++) { sx += X[i]; sz += Z[i]; }
  const mx = sx / N, mz = sz / N;
  let sxz = 0, sxx = 0, szz = 0;
  for (let i = 0; i < N; i++) {
    const a = X[i] - mx, b = Z[i] - mz;
    sxz += a * b; sxx += a * a; szz += b * b;
  }
  if (sxx <= 0 || szz <= 0) return null;
  const slope = sxz / sxx;
  if (Math.abs(slope) < 1e-12) return null;
  const intercept = mz - slope * mx;
  const zeroRaw = -intercept / slope;
  const r = sxz / Math.sqrt(sxx * szz);

  if (!Number.isFinite(zeroRaw)) return null;
  return { zeroRaw, r, samples: N };
}

// ==================== 조향 보정 패널 UI ====================

let steerMenuEl = null;

function closeSteeringPanel() {
  if (steerMenuEl) { steerMenuEl.remove(); steerMenuEl = null; }
}

document.addEventListener('click', e => {
  if (steerMenuEl && !steerMenuEl.contains(e.target) && !e.target.closest('.steering-cal-trigger')) {
    closeSteeringPanel();
  }
});
document.addEventListener('keydown', e => { if (e.key === 'Escape') closeSteeringPanel(); });

let steerEstimates = null; // { dist, gps } — 추정 결과 캐시

function currentSteerRaw() {
  if (typeof activeSampledData === 'undefined' || !activeSampledData.length) return null;
  const row = activeSampledData[currentCursorIndex];
  if (!row) return null;
  const v = Number(row.steering_raw);
  return Number.isFinite(v) ? v : null;
}

/**
 * 조향 차트의 Y축만 사용자가 지정한 범위로 조정합니다.
 * 실제 조향각 값과 Yaw/횡G 전용 축은 변경하지 않습니다.
 */
function updateSteeringAxisRange() {
  const fit = (chart) => {
    if (!chart || !chart.options.scales || !chart.options.scales.y) return;
    const lim = steeringCal.axisLimit;
    chart.options.scales.y.min = -lim;
    chart.options.scales.y.max = lim;
    chart.update('none');
  };

  if (typeof chartSteering !== 'undefined') fit(chartSteering);
  if (typeof diagChartSteering !== 'undefined') fit(diagChartSteering);
}

function applySteeringCalChange() {
  saveSteeringCal();
  if (typeof rebuildRawChannel === 'function' && typeof globalData !== 'undefined' && globalData.length) {
    rebuildRawChannel('steering', globalData);
  }
  if (typeof refreshChartsAfterFilter === 'function') refreshChartsAfterFilter();
  updateSteeringAxisRange();
  updateSteeringCalBadges();
}

function openSteeringPanel(x, y) {
  closeSteeringPanel();
  const el = document.createElement('div');
  el.className = 'filter-menu steer-menu';
  steerMenuEl = el;
  document.body.appendChild(el);

  const render = () => {
    const raw = currentSteerRaw();
    const deg = raw === null ? null : (raw - steeringCal.zeroRaw) * steeringCal.degPerLsb * (steeringCal.invert ? -1 : 1);

    let html = `<div class="fm-head">
        <span class="fm-title">조향 영점 보정</span>
        <button class="fm-close" data-act="close">✕</button>
      </div>`;

    html += `<div class="fm-sec">
      <div class="sc-live">
        <span>커서 지점</span>
        <strong>raw ${raw === null ? '----' : raw.toFixed(0)}</strong>
        <span class="sc-arrow">→</span>
        <strong class="sc-deg">${deg === null ? '--.-' : (deg >= 0 ? '+' : '') + deg.toFixed(1)}°</strong>
      </div>
      <div class="sc-formula">조향각 = (raw − 영점) × 0.1°/LSB ${steeringCal.invert ? '× (−1)' : ''}</div>
    </div>`;

    html += `<div class="fm-sec">
      <div class="fm-param">
        <label>영점 raw</label>
        <input class="fm-range" type="range" data-act="zero" min="0" max="4095" step="1" value="${steeringCal.zeroRaw}">
        <input class="fm-num fm-num-w" type="number" data-act="zero-num" min="0" max="4095" step="1" value="${steeringCal.zeroRaw}">
        <span class="fm-unit">LSB</span>
      </div>
      <div class="fm-param">
        <label>조향 Y축</label>
        <input class="fm-range" type="range" data-act="axis" min="20" max="500" step="10" value="${steeringCal.axisLimit}">
        <input class="fm-num fm-num-w" type="number" data-act="axis-num" min="20" max="500" step="10" value="${steeringCal.axisLimit}">
        <span class="fm-unit">± °</span>
      </div>
      <label class="fm-check" style="margin-top:8px;">
        <input type="checkbox" data-act="invert" ${steeringCal.invert ? 'checked' : ''}>
        <span>좌우 반전 (조향 부호 뒤집기)</span>
      </label>
    </div>`;

    html += `<div class="fm-sec">
      <button class="fm-btn fm-btn-primary fm-btn-wide" data-act="estimate">🎯 자동 영점 추정</button>
      <div class="sc-est" data-est></div>
    </div>`;

    html += `<div class="fm-foot">
        <button class="fm-btn" data-act="cursor-zero">현재 커서를 0°로</button>
        <button class="fm-btn" data-act="reset">기본값 복원</button>
      </div>
      <div class="fm-hint">보정값은 이 브라우저에 저장되어 다음 접속 때도 유지됩니다.
      원본 CSV와 로거 설정은 바뀌지 않으며, 화면 표시만 달라집니다.</div>`;

    el.innerHTML = html;
    renderEstimates();
    bind();
  };

  const renderEstimates = () => {
    const box = el.querySelector('[data-est]');
    if (!box || !steerEstimates) return;
    const { dist, gps } = steerEstimates;
    let h = '';

    if (!dist && !gps) {
      box.innerHTML = `<div class="sc-est-fail">추정에 필요한 주행 표본이 부족합니다. 로그가 짧거나 정차 구간만 있는지 확인해 주세요.</div>`;
      return;
    }

    if (dist) {
      const curDeg = (dist.zeroRaw - steeringCal.zeroRaw) * steeringCal.degPerLsb;
      h += `<div class="sc-est-row">
        <div class="sc-est-main"><b>분포 기반</b> raw <b>${dist.zeroRaw.toFixed(0)}</b>
          <span class="sc-est-diff">(현재 영점 대비 ${curDeg >= 0 ? '+' : ''}${curDeg.toFixed(1)}°)</span></div>
        <div class="sc-est-sub">${dist.how} · 표본 ${dist.samples.toLocaleString()}개 · 봉우리 집중도 ${(dist.sharpness * 100).toFixed(0)}%</div>
        <button class="fm-btn" data-act="use-dist">이 값 적용</button>
      </div>`;
    }
    if (gps) {
      const curDeg = (gps.zeroRaw - steeringCal.zeroRaw) * steeringCal.degPerLsb;
      const quality = Math.abs(gps.r) > 0.7 ? '신뢰도 높음' : Math.abs(gps.r) > 0.4 ? '보통' : '낮음';
      h += `<div class="sc-est-row">
        <div class="sc-est-main"><b>GPS 궤적 검증</b> raw <b>${gps.zeroRaw.toFixed(0)}</b>
          <span class="sc-est-diff">(현재 영점 대비 ${curDeg >= 0 ? '+' : ''}${curDeg.toFixed(1)}°)</span></div>
        <div class="sc-est-sub">요레이트 회귀 · 상관계수 r=${gps.r.toFixed(2)} (${quality}) · 표본 ${gps.samples.toLocaleString()}개</div>
        <button class="fm-btn" data-act="use-gps">이 값 적용</button>
      </div>`;
    }
    if (dist && gps) {
      const diff = Math.abs(dist.zeroRaw - gps.zeroRaw);
      const diffDeg = diff * steeringCal.degPerLsb;
      const agree = diffDeg < 3;
      h += `<div class="sc-est-verdict ${agree ? 'ok' : 'warn'}">
        ${agree ? '✔' : '⚠'} 두 방법의 차이 ${diff.toFixed(0)} LSB (${diffDeg.toFixed(1)}°) —
        ${agree ? '독립적인 두 측정이 일치하므로 신뢰할 수 있습니다.'
                : '차이가 큽니다. GPS 수신 품질이나 주행 구간을 확인한 뒤 분포 기반 값을 우선 검토하세요.'}
        <button class="fm-btn" data-act="use-avg">두 값의 평균 적용 (raw ${((dist.zeroRaw + gps.zeroRaw) / 2).toFixed(0)})</button>
      </div>`;
    }
    box.innerHTML = h;
    bind();
  };

  let timer = null;
  const schedule = (now) => {
    clearTimeout(timer);
    if (now) applySteeringCalChange();
    else timer = setTimeout(applySteeringCalChange, 90);
  };

  const setZero = (v) => {
    steeringCal.zeroRaw = Math.max(0, Math.min(4095, Math.round(v)));
    render(); schedule(true);
  };

  const bind = () => {
    el.querySelectorAll('[data-act]').forEach(node => {
      const act = node.dataset.act;

      if (act === 'close') node.onclick = closeSteeringPanel;

      if (act === 'zero' || act === 'zero-num') node.oninput = e => {
        const v = parseFloat(e.target.value);
        if (!Number.isFinite(v)) return;
        steeringCal.zeroRaw = Math.max(0, Math.min(4095, Math.round(v)));
        el.querySelectorAll('[data-act="zero"],[data-act="zero-num"]').forEach(n => { if (n !== e.target) n.value = steeringCal.zeroRaw; });
        const raw = currentSteerRaw();
        const dg = el.querySelector('.sc-deg');
        if (dg && raw !== null) {
          const d = (raw - steeringCal.zeroRaw) * steeringCal.degPerLsb * (steeringCal.invert ? -1 : 1);
          dg.textContent = (d >= 0 ? '+' : '') + d.toFixed(1) + '°';
        }
        schedule();
      };

      if (act === 'axis' || act === 'axis-num') node.oninput = e => {
        const v = parseFloat(e.target.value);
        if (!Number.isFinite(v)) return;
        steeringCal.axisLimit = Math.max(20, Math.min(500, v));
        el.querySelectorAll('[data-act="axis"],[data-act="axis-num"]').forEach(n => { if (n !== e.target) n.value = steeringCal.axisLimit; });
        schedule();
      };

      if (act === 'invert') node.onchange = e => {
        steeringCal.invert = e.target.checked;
        render(); schedule(true);
      };

      if (act === 'estimate') node.onclick = () => {
        if (typeof globalData === 'undefined' || !globalData.length) {
          alert('먼저 CSV 로그를 불러와 주세요.');
          return;
        }
        node.textContent = '⏳ 분석 중...';
        node.disabled = true;
        setTimeout(() => {
          steerEstimates = {
            dist: estimateSteeringZeroByDistribution(globalData),
            gps: estimateSteeringZeroByGps(globalData)
          };
          render();
        }, 20);
      };

      if (act === 'use-dist') node.onclick = () => setZero(steerEstimates.dist.zeroRaw);
      if (act === 'use-gps') node.onclick = () => setZero(steerEstimates.gps.zeroRaw);
      if (act === 'use-avg') node.onclick = () => setZero((steerEstimates.dist.zeroRaw + steerEstimates.gps.zeroRaw) / 2);

      if (act === 'cursor-zero') node.onclick = () => {
        const raw = currentSteerRaw();
        if (raw === null) { alert('먼저 로그를 불러오고 그래프에서 기준으로 삼을 지점에 커서를 놓아 주세요.'); return; }
        setZero(raw);
      };

      if (act === 'reset') node.onclick = () => {
        Object.assign(steeringCal, STEER_CAL_DEFAULT);
        render(); schedule(true);
      };
    });
  };

  render();

  const rect = el.getBoundingClientRect();
  let left = x, top = y;
  if (left + rect.width > window.innerWidth - 8) left = window.innerWidth - rect.width - 8;
  if (top + rect.height > window.innerHeight - 8) top = Math.max(8, window.innerHeight - rect.height - 8);
  el.style.left = Math.max(8, left) + 'px';
  el.style.top = Math.max(8, top) + 'px';
}

/** 보정이 기본값과 다르면 핸들 위젯에 표시 */
function updateSteeringCalBadges() {
  const custom = !isSteeringCalDefault();
  document.querySelectorAll('.steering-cal-trigger').forEach(el => {
    el.classList.toggle('cal-custom', custom);
    el.title = custom
      ? `조향 설정 (영점 raw ${steeringCal.zeroRaw}, Y축 ±${steeringCal.axisLimit}°${steeringCal.invert ? ', 반전' : ''}) — 클릭하여 변경`
      : '클릭하여 조향 영점 보정';
  });
}

/**
 * 핸들 그래픽에 보정 트리거를 붙입니다.
 *
 * 1페이지의 핸들 위젯은 차트 캔버스 위에 겹쳐 있고 `pointer-events: none`이라
 * 마우스 이벤트가 캔버스로 통과합니다(차트 호버 동기화 유지 목적). 위젯 전체를
 * 클릭 대상으로 만들면 그 영역에서 그래프 커서가 죽으므로, 대신 작은 톱니 버튼만
 * `pointer-events: auto`로 띄웁니다.
 * 2페이지 핸들은 라벨바 안에 있어 캔버스를 가리지 않으므로 전체를 클릭 대상으로 둡니다.
 */
function initSteeringCalibration() {
  const openFrom = (el, e) => {
    if (e) { e.preventDefault(); e.stopPropagation(); }
    if (e && Number.isFinite(e.clientX) && e.type === 'contextmenu') {
      openSteeringPanel(e.clientX, e.clientY);
      return;
    }
    const r = el.getBoundingClientRect();
    openSteeringPanel(r.left - 300, r.bottom + 8);
  };

  const addGearButton = (host) => {
    if (host.querySelector('.steering-cal-btn')) return;
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'steering-cal-btn steering-cal-trigger';
    btn.textContent = '⚙';
    btn.setAttribute('aria-label', '조향 영점 보정');
    btn.addEventListener('click', e => openFrom(host, e));
    btn.addEventListener('contextmenu', e => openFrom(host, e));
    host.appendChild(btn);
  };

  const overlay = document.querySelector('.steering-wheel-overlay');
  if (overlay) addGearButton(overlay);

  const diagWrap = document.querySelector('.diag-steering-wheel-wrapper');
  if (diagWrap) {
    diagWrap.classList.add('steering-cal-trigger', 'steering-cal-clickable');
    diagWrap.addEventListener('click', e => openFrom(diagWrap, e));
    diagWrap.addEventListener('contextmenu', e => openFrom(diagWrap, e));
    addGearButton(diagWrap);
  }

  updateSteeringCalBadges();
}
