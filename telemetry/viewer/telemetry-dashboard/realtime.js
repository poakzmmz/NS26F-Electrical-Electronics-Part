/* =============================================================================
 * NS26F Telemetry — 5번 탭: 실시간 무선 텔레메트리
 * -----------------------------------------------------------------------------
 * 구성:
 *   차량 ──RF(LoRa)──▶ 수신 PC(중계 서버) ──WebSocket/SSE──▶ 이 웹페이지 (여러 팀원)
 *
 * 이 페이지는 순수 정적 사이트라 자체 서버가 없습니다. RF를 받는 PC가
 * tools/rf_bridge.py 같은 중계 서버를 띄우고 그 주소를 입력하면, 접속한
 * 팀원 전원이 같은 값을 실시간으로 보게 됩니다.
 *
 * ── 수신감도 6단계 기준 (telemetry_20260801_095829.csv 8,300패킷 실측 기반) ──
 *  실제 RF 로그에서 seq 연속성으로 패킷 손실을 계산하고 RSSI/SNR과 대조한 결과,
 *  SNR이 RSSI보다 훨씬 좋은 예측 지표였습니다. (5초 윈도우 손실률 중앙값/p90)
 *
 *    SNR  ≥ 8.0 dB  → 손실  0.0% / p90  4.0%      RSSI ≥ -95  → 0.0% / 4.0%
 *    SNR  6.0~8.0   → 손실  0.0% / p90  8.3%      RSSI -100~-95 → 0.0% / 5.6%
 *    SNR  3.0~6.0   → 손실  0.0% / p90 23.9%      RSSI -105~-100 → 0.0% / 21.8%
 *    SNR  0.0~3.0   → 손실 16.7% / p90 45.7%      RSSI -110~-105 → 19.2% / 57.8%
 *    SNR  < 0.0     → 손실 24~35% / p90 54~70%    RSSI < -110  → 24.0% / 46.6%
 *
 *  → SNR 3dB / RSSI -105dBm 부근에서 손실률이 급격히 꺾이는 절벽이 존재.
 *
 *  무수신 판정: 공칭 수신 주기가 236ms(4.2Hz)이고 실주행 중 3초 이상 공백은
 *  전체의 0.1~0.22%에 불과했으므로, 3초 무수신을 '신호없음'으로 잡으면
 *  오탐이 거의 없습니다. 1.5초 이상은 '끊김' 경고로 처리합니다.
 *
 *  최종 등급 = (실측 손실률 / SNR / RSSI) 세 지표 중 가장 나쁜 값.
 *  링크가 무너지기 직전을 미리 경고하기 위해 보수적으로 판정합니다.
 * ========================================================================== */

// ==================== 등급 정의 ====================

const RT_GRADES = ['신호없음', '아주나쁨', '나쁨', '보통', '좋음', '아주좋음'];
const RT_GRADE_KEY = ['none', 'vbad', 'bad', 'fair', 'good', 'vgood'];

/** 실측 패킷 손실률(%) → 등급 인덱스 */
function rtGradeFromLoss(lossPct) {
  if (lossPct < 2) return 5;
  if (lossPct < 5) return 4;
  if (lossPct < 15) return 3;
  if (lossPct < 30) return 2;
  return 1;
}

/** SNR(dB) → 등급 인덱스 (LoRa 링크 마진) */
function rtGradeFromSnr(snr) {
  if (snr >= 8) return 5;
  if (snr >= 6) return 4;
  if (snr >= 3) return 3;
  if (snr >= 0) return 2;
  return 1;
}

/** RSSI(dBm) → 등급 인덱스 */
function rtGradeFromRssi(rssi) {
  if (rssi >= -95) return 5;
  if (rssi >= -100) return 4;
  if (rssi >= -105) return 3;
  if (rssi >= -110) return 2;
  return 1;
}

const RT_STALE_WARN_MS = 1500;   // 끊김 경고
const RT_STALE_NONE_MS = 3000;   // 신호없음 판정

// ==================== 채널 레지스트리 ====================
// 실시간 스트림에 등장하는 필드명 → 표시 정보.
// 등록되지 않은 숫자 필드가 들어오면 자동으로 카드가 추가됩니다.

const RT_CHANNELS = {
  rpm:         { label: 'Engine RPM',   unit: 'rpm',  color: '#dc2626', decimals: 0, min: 0, max: 12000, order: 1 },
  vss_kmh:     { label: 'Speed',        unit: 'km/h', color: '#f97316', decimals: 1, min: 0, order: 2, alias: ['speed_kmh', 'fl_speed_kmh', 'can_speed_kmh'] },
  tps_pct:     { label: 'Throttle',     unit: '%',    color: '#16a34a', decimals: 1, min: 0, max: 100, order: 3, alias: ['tps', 'throttle_pct'] },
  brake_pct:   { label: 'Brake',        unit: '%',    color: '#dc2626', decimals: 1, min: 0, max: 100, order: 4 },
  gear:        { label: 'Gear',         unit: '',     color: '#2563eb', decimals: 0, min: 0, max: 6, order: 5, stepped: true, gearText: true },
  steering_deg:{ label: 'Steering',     unit: '°',    color: '#db2777', decimals: 1, order: 6, alias: ['steering', 'steer_deg'] },
  clt_c:       { label: 'Coolant Temp', unit: '°C',   color: '#2563eb', decimals: 0, order: 7, alias: ['water_c', 'coolant_c'], warnAbove: 105 },
  oil_c:       { label: 'Oil Temp',     unit: '°C',   color: '#f97316', decimals: 0, order: 8, warnAbove: 130 },
  iat_c:       { label: 'Intake Air',   unit: '°C',   color: '#16a34a', decimals: 0, order: 9 },
  ecu_c:       { label: 'ECU Temp',     unit: '°C',   color: '#db2777', decimals: 0, order: 10 },
  batt_v:      { label: 'Battery',      unit: 'V',    color: '#eab308', decimals: 2, order: 11, alias: ['battery_v'], warnBelow: 11.5 },
  fuel_used_l: { label: 'Fuel Used',    unit: 'L',    color: '#8b5cf6', decimals: 2, min: 0, order: 12 },
  rssi:        { label: 'RSSI',         unit: 'dBm',  color: '#06b6d4', decimals: 0, order: 90, link: true },
  snr:         { label: 'SNR',          unit: 'dB',   color: '#10b981', decimals: 1, order: 91, link: true }
};

// 표시하지 않을 메타 필드
const RT_META_FIELDS = new Set([
  'seq', 'msg_type', 'vehicle_id', 'timestamp_iso', 'elapsed_s', 'ts', 'time', 't', 'type', 'id'
]);

/** alias → 표준 채널키 매핑 테이블 */
const RT_ALIAS = (() => {
  const m = {};
  Object.entries(RT_CHANNELS).forEach(([k, c]) => {
    m[k] = k;
    (c.alias || []).forEach(a => { m[a] = k; });
  });
  return m;
})();

// ==================== 상태 ====================

const RT_WINDOW_SEC = 60;        // 그래프에 보여줄 시간 창
const RT_MAX_POINTS = 900;       // 채널당 최대 포인트 (4.2Hz × 60s ≈ 250, 여유 확보)

const rtState = {
  connected: false,
  mode: null,                    // 'ws' | 'sse' | 'poll' | 'demo'
  url: '',
  socket: null,
  es: null,
  pollTimer: null,
  demoTimer: null,
  reconnectTimer: null,
  reconnectDelay: 1000,
  manualClose: false,

  packets: 0,
  lastPacketAt: 0,
  startedAt: 0,
  // msg_type별 seq 추적 (FAST/SLOW처럼 종류별로 시퀀스가 따로 도는 경우 대응)
  seqTrack: {},
  // 최근 수신 이력 (손실률/수신률 계산용) [{t, lost, type}]
  history: [],
  latest: {},                    // 채널키 → 최신 값
  series: {},                    // 채널키 → {t:[], v:[]}
  charts: {},                    // 채널키 → Chart 인스턴스
  cards: {},                     // 채널키 → DOM
  order: [],
  csvHeader: null,
  lastError: null
};

// ==================== DOM ====================

const rtEl = {};
function rtCacheDom() {
  ['rt-url', 'rt-transport', 'rt-connect', 'rt-demo', 'rt-grid', 'rt-empty',
   'rt-signal', 'rt-signal-grade', 'rt-signal-reason', 'rt-signal-bars',
   'rt-stat-rate', 'rt-stat-loss', 'rt-stat-rssi', 'rt-stat-snr', 'rt-stat-age', 'rt-stat-count'
  ].forEach(id => { rtEl[id] = document.getElementById(id); });
}

// ==================== 메시지 파싱 ====================

/**
 * 수신 문자열을 텔레메트리 레코드 배열로 변환합니다.
 * 지원 형식:
 *   1) JSON 객체            {"rpm":5000,"tps_pct":30,...}
 *   2) JSON 배열(묶음 전송)   [{...},{...}]
 *   3) NDJSON               한 줄에 JSON 하나씩
 *   4) CSV 헤더 선언         {"header":["rpm","tps_pct",...]}  또는  "#rpm,tps_pct,..."
 *   5) CSV 데이터 줄         헤더가 먼저 선언된 경우에 한해 파싱
 */
function rtParseMessage(text) {
  const out = [];
  if (typeof text !== 'string') {
    if (text && typeof text === 'object') return Array.isArray(text) ? text : [text];
    return out;
  }
  const lines = text.split('\n');
  for (let raw of lines) {
    const line = raw.trim();
    if (!line) continue;

    if (line.startsWith('#')) {                 // CSV 헤더 선언
      rtState.csvHeader = line.slice(1).split(',').map(s => s.trim());
      continue;
    }
    if (line[0] === '{' || line[0] === '[') {
      try {
        const o = JSON.parse(line);
        if (Array.isArray(o)) { o.forEach(x => out.push(x)); continue; }
        if (o && Array.isArray(o.header)) { rtState.csvHeader = o.header.map(s => String(s).trim()); continue; }
        if (o && Array.isArray(o.data)) { o.data.forEach(x => out.push(x)); continue; }
        out.push(o);
      } catch (err) { /* JSON이 아니면 CSV로 시도 */ }
      continue;
    }
    if (rtState.csvHeader) {                    // CSV 데이터 줄
      const p = line.split(',');
      // 필드 개수가 헤더와 크게 다르면 텔레메트리 줄이 아니라고 보고 버립니다.
      // (수신기가 찍는 로그/에러 메시지가 섞여 들어오는 것을 막기 위함)
      if (p.length < rtState.csvHeader.length - 1 || p.length > rtState.csvHeader.length + 1) continue;
      const o = {};
      let numericCount = 0;
      rtState.csvHeader.forEach((h, i) => {
        const v = p[i];
        if (v === undefined || v === '') return;
        const t = v.trim();
        const n = Number(t);
        if (t !== '' && Number.isFinite(n)) { o[h] = n; numericCount++; }
        else o[h] = t;
      });
      if (numericCount > 0) out.push(o);
    }
  }
  return out;
}

// ==================== 채널 감지 및 카드 생성 ====================

function rtChannelInfo(rawKey) {
  // CSV 헤더에 딸려 오는 공백/개행이 채널키에 섞이지 않도록 정규화
  const key = String(rawKey).trim();
  const std = RT_ALIAS[key] || key;
  const def = RT_CHANNELS[std];
  if (def) return { key: std, def };
  return {
    key: std,
    def: { label: std, unit: '', color: '#94a3b8', decimals: 2, order: 50, auto: true }
  };
}

function rtEnsureCard(key) {
  if (rtState.cards[key]) return rtState.cards[key];
  const { def } = rtChannelInfo(key);

  if (rtEl['rt-empty']) rtEl['rt-empty'].style.display = 'none';

  const card = document.createElement('div');
  card.className = 'rt-card' + (def.link ? ' rt-card-link' : '');
  card.dataset.ch = key;
  card.style.order = String(def.order ?? 50);
  card.innerHTML = `
    <div class="rt-card-head">
      <span class="rt-card-title">${def.label}</span>
      <span class="rt-card-unit">${def.unit || ''}</span>
    </div>
    <div class="rt-card-value" data-v>--</div>
    <div class="rt-card-chart"><canvas></canvas></div>`;
  rtEl['rt-grid'].appendChild(card);

  const canvas = card.querySelector('canvas');
  const chart = new Chart(canvas.getContext('2d'), {
    type: 'line',
    data: {
      datasets: [{
        data: [],
        borderColor: def.color,
        backgroundColor: def.color + '22',
        borderWidth: 1.6,
        pointRadius: 0,
        fill: true,
        stepped: def.stepped ? 'before' : false,
        tension: def.stepped ? 0 : 0.15
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      normalized: true,
      spanGaps: true,
      plugins: { legend: { display: false }, tooltip: { enabled: false } },
      layout: { padding: { top: 2, bottom: 0, left: 0, right: 0 } },
      scales: {
        x: { type: 'linear', display: false },
        y: {
          display: true,
          min: def.min,
          max: def.max,
          grid: { color: 'rgba(128,128,128,0.10)' },
          ticks: { color: 'rgba(128,128,128,0.75)', font: { family: 'JetBrains Mono', size: 8 }, maxTicksLimit: 3 },
          afterFit(axis) { axis.width = 34; }
        }
      }
    }
  });

  rtState.cards[key] = { card, chart, valueEl: card.querySelector('[data-v]'), def };
  rtState.series[key] = { t: [], v: [] };
  return rtState.cards[key];
}

// ==================== 패킷 수신 처리 ====================

function rtIngest(rec) {
  if (!rec || typeof rec !== 'object') return;
  const now = performance.now();

  // --- 시퀀스 기반 손실 추적 (msg_type별로 분리) ---
  const type = rec.msg_type || rec.type || '_';
  let lost = 0;
  if (Number.isFinite(Number(rec.seq))) {
    const seq = Number(rec.seq) | 0;
    const prev = rtState.seqTrack[type];
    if (prev !== undefined) {
      // 8비트 순환 seq 가정, 40 이상 점프는 장기 단절로 보고 손실 계산에서 제외
      const gap = ((seq - prev) % 256 + 256) % 256;
      if (gap >= 1 && gap <= 40) lost = gap - 1;
    }
    rtState.seqTrack[type] = seq;
  }

  rtState.packets++;
  rtState.lastPacketAt = now;
  if (!rtState.startedAt) rtState.startedAt = now;
  rtState.history.push({ t: now, lost });
  const cutoff = now - 10000;
  while (rtState.history.length && rtState.history[0].t < cutoff) rtState.history.shift();

  // --- 채널 값 반영 ---
  const tSec = now / 1000;
  Object.keys(rec).forEach(rawField => {
    const field = String(rawField).trim();
    if (RT_META_FIELDS.has(field)) return;
    const val = rec[rawField];
    if (val === null || val === undefined || val === '') return;
    const num = Number(val);
    if (!Number.isFinite(num)) return;

    const { key } = rtChannelInfo(field);
    rtState.latest[key] = num;

    rtEnsureCard(key);
    const s = rtState.series[key];
    s.t.push(tSec); s.v.push(num);
    const tCut = tSec - RT_WINDOW_SEC;
    let drop = 0;
    while (drop < s.t.length && s.t[drop] < tCut) drop++;
    if (drop > 0) { s.t.splice(0, drop); s.v.splice(0, drop); }
    if (s.t.length > RT_MAX_POINTS) {
      const ex = s.t.length - RT_MAX_POINTS;
      s.t.splice(0, ex); s.v.splice(0, ex);
    }
  });

  rtState.dirty = true;
}

// ==================== 링크 품질 판정 ====================

function rtComputeLink() {
  const now = performance.now();
  const age = rtState.lastPacketAt ? now - rtState.lastPacketAt : Infinity;

  if (!rtState.connected && !rtState.lastPacketAt) {
    return { grade: 0, reason: '연결되지 않음', rate: 0, loss: null, age: null };
  }
  if (age > RT_STALE_NONE_MS) {
    return {
      grade: 0,
      reason: rtState.connected ? `${(age / 1000).toFixed(1)}초째 수신 없음` : '연결 끊김',
      rate: 0, loss: null, age
    };
  }

  // 최근 5초 수신률 / 손실률
  const win = 5000;
  const cut = now - win;
  let recv = 0, lost = 0;
  for (let i = rtState.history.length - 1; i >= 0; i--) {
    const h = rtState.history[i];
    if (h.t < cut) break;
    recv++; lost += h.lost;
  }
  const elapsed = Math.min(win, now - rtState.startedAt) / 1000;
  const rate = elapsed > 0.5 ? recv / elapsed : 0;
  const loss = (recv + lost) > 0 ? (100 * lost) / (recv + lost) : 0;

  const reasons = [];
  let grade = 5;

  if (recv + lost >= 8) {
    const g = rtGradeFromLoss(loss);
    if (g < grade) { grade = g; }
    reasons.push(`손실 ${loss.toFixed(1)}%`);
  }

  const snr = rtState.latest.snr;
  if (Number.isFinite(snr)) {
    const g = rtGradeFromSnr(snr);
    if (g < grade) grade = g;
    reasons.push(`SNR ${snr.toFixed(1)}dB`);
  }

  const rssi = rtState.latest.rssi;
  if (Number.isFinite(rssi)) {
    const g = rtGradeFromRssi(rssi);
    if (g < grade) grade = g;
    reasons.push(`RSSI ${rssi.toFixed(0)}dBm`);
  }

  if (age > RT_STALE_WARN_MS) {
    grade = Math.min(grade, 1);
    reasons.unshift(`${(age / 1000).toFixed(1)}초 끊김`);
  }

  return { grade, reason: reasons.join(' · ') || '수신 중', rate, loss, age };
}

// ==================== 화면 갱신 ====================

let rtRafPending = false;

function rtScheduleRender() {
  if (rtRafPending) return;
  rtRafPending = true;
  requestAnimationFrame(() => {
    rtRafPending = false;
    rtRender();
  });
}

function rtRender() {
  const active = pageRealtime && pageRealtime.classList.contains('active');

  // 링크 상태는 탭이 꺼져 있어도 계속 갱신 (헤더 배지 용도)
  const link = rtComputeLink();
  rtRenderLink(link);

  if (!active) return;

  Object.entries(rtState.cards).forEach(([key, c]) => {
    const v = rtState.latest[key];
    if (Number.isFinite(v)) {
      if (c.def.gearText) c.valueEl.textContent = v === 0 ? 'N' : String(Math.round(v));
      else c.valueEl.textContent = v.toFixed(c.def.decimals ?? 1);

      const warn = (c.def.warnAbove !== undefined && v > c.def.warnAbove) ||
                   (c.def.warnBelow !== undefined && v < c.def.warnBelow);
      c.card.classList.toggle('rt-warn', !!warn);
    }

    const s = rtState.series[key];
    if (!s || !s.t.length) return;
    const pts = new Array(s.t.length);
    for (let i = 0; i < s.t.length; i++) pts[i] = { x: s.t[i], y: s.v[i] };
    c.chart.data.datasets[0].data = pts;
    const tEnd = s.t[s.t.length - 1];
    c.chart.options.scales.x.min = tEnd - RT_WINDOW_SEC;
    c.chart.options.scales.x.max = tEnd;
    c.chart.update('none');
  });
}

function rtRenderLink(link) {
  const g = link.grade;
  if (rtEl['rt-signal']) {
    rtEl['rt-signal'].className = 'rt-signal rt-grade-' + RT_GRADE_KEY[g];
  }
  if (rtEl['rt-signal-grade']) rtEl['rt-signal-grade'].textContent = RT_GRADES[g];
  if (rtEl['rt-signal-reason']) rtEl['rt-signal-reason'].textContent = link.reason;
  if (rtEl['rt-signal-bars']) {
    rtEl['rt-signal-bars'].querySelectorAll('span').forEach((b, i) => {
      b.classList.toggle('on', i < g);
    });
  }
  if (rtEl['rt-stat-rate']) rtEl['rt-stat-rate'].textContent = link.rate.toFixed(1) + ' Hz';
  if (rtEl['rt-stat-loss']) rtEl['rt-stat-loss'].textContent = link.loss === null ? '--' : link.loss.toFixed(1) + ' %';
  if (rtEl['rt-stat-rssi']) {
    const v = rtState.latest.rssi;
    rtEl['rt-stat-rssi'].textContent = Number.isFinite(v) ? v.toFixed(0) + ' dBm' : '-- dBm';
  }
  if (rtEl['rt-stat-snr']) {
    const v = rtState.latest.snr;
    rtEl['rt-stat-snr'].textContent = Number.isFinite(v) ? v.toFixed(1) + ' dB' : '-- dB';
  }
  if (rtEl['rt-stat-age']) {
    rtEl['rt-stat-age'].textContent = link.age === null || !Number.isFinite(link.age)
      ? '--' : (link.age / 1000).toFixed(1) + '초 전';
  }
  if (rtEl['rt-stat-count']) rtEl['rt-stat-count'].textContent = rtState.packets.toLocaleString();
}

// ==================== 연결 관리 ====================

function rtSetConnState(on, label) {
  rtState.connected = on;
  if (rtEl['rt-connect']) {
    rtEl['rt-connect'].textContent = label || (on ? '연결 해제' : '연결');
    rtEl['rt-connect'].classList.toggle('rt-btn-primary', !on);
    rtEl['rt-connect'].classList.toggle('rt-btn-danger', on);
  }
}

function rtDetectTransport(url) {
  const sel = rtEl['rt-transport'] ? rtEl['rt-transport'].value : 'auto';
  if (sel !== 'auto') return sel;
  if (/^wss?:\/\//i.test(url)) return 'ws';
  if (/\/(events|sse|stream)\b/i.test(url)) return 'sse';
  return 'poll';
}

function rtDisconnect(manual) {
  rtState.manualClose = !!manual;
  clearTimeout(rtState.reconnectTimer);
  clearInterval(rtState.pollTimer);
  clearInterval(rtState.demoTimer);
  rtState.pollTimer = null;
  rtState.demoTimer = null;
  if (rtState.socket) { try { rtState.socket.close(); } catch (e) { /* ignore */ } rtState.socket = null; }
  if (rtState.es) { try { rtState.es.close(); } catch (e) { /* ignore */ } rtState.es = null; }
  rtSetConnState(false);
  rtScheduleRender();
}

function rtScheduleReconnect() {
  if (rtState.manualClose || rtState.mode === 'demo') return;
  clearTimeout(rtState.reconnectTimer);
  const d = rtState.reconnectDelay;
  if (rtEl['rt-signal-reason']) {
    rtEl['rt-signal-reason'].textContent = `재연결 대기 ${(d / 1000).toFixed(0)}초...`;
  }
  rtState.reconnectTimer = setTimeout(() => {
    rtConnect(rtState.url, true);
  }, d);
  rtState.reconnectDelay = Math.min(15000, Math.round(d * 1.7));
}

function rtConnect(url, isRetry) {
  if (!url) { alert('수신 서버 주소를 입력해 주세요.\n예) ws://192.168.0.10:8765'); return; }
  if (!isRetry) { rtDisconnect(true); rtState.reconnectDelay = 1000; }
  rtState.manualClose = false;
  rtState.url = url;
  try { localStorage.setItem('nssur_rt_url', url); } catch (e) { /* ignore */ }

  const mode = rtDetectTransport(url);
  rtState.mode = mode;
  rtSetConnState(true, '연결 해제');
  if (rtEl['rt-signal-reason']) rtEl['rt-signal-reason'].textContent = '연결 시도 중...';

  if (mode === 'ws') {
    let sock;
    try { sock = new WebSocket(url); }
    catch (err) { rtState.lastError = err.message; rtScheduleReconnect(); return; }
    rtState.socket = sock;
    sock.onopen = () => {
      rtState.reconnectDelay = 1000;
      if (rtEl['rt-signal-reason']) rtEl['rt-signal-reason'].textContent = '연결됨 · 첫 패킷 대기';
    };
    sock.onmessage = ev => { rtParseMessage(ev.data).forEach(rtIngest); rtScheduleRender(); };
    sock.onerror = () => { rtState.lastError = 'WebSocket 오류'; };
    sock.onclose = () => { rtState.socket = null; if (!rtState.manualClose) rtScheduleReconnect(); };

  } else if (mode === 'sse') {
    let es;
    try { es = new EventSource(url); }
    catch (err) { rtState.lastError = err.message; rtScheduleReconnect(); return; }
    rtState.es = es;
    es.onopen = () => { rtState.reconnectDelay = 1000; };
    es.onmessage = ev => { rtParseMessage(ev.data).forEach(rtIngest); rtScheduleRender(); };
    es.onerror = () => {
      rtState.lastError = 'SSE 오류';
      try { es.close(); } catch (e) { /* ignore */ }
      rtState.es = null;
      if (!rtState.manualClose) rtScheduleReconnect();
    };

  } else {
    // HTTP 폴링. 서버가 {"cursor":N,"data":[...]} 형태로 응답하면 커서를 보내
    // 마지막 폴링 이후 새로 도착한 패킷만 받습니다(중복 방지).
    // 커서를 모르는 단순 서버라면 응답 전체를 그대로 해석합니다.
    rtState.pollCursor = null;
    const poll = () => {
      let u = url;
      if (rtState.pollCursor !== null) {
        u += (url.includes('?') ? '&' : '?') + 'since=' + encodeURIComponent(rtState.pollCursor);
      }
      fetch(u, { cache: 'no-store' })
        .then(r => { if (!r.ok) throw new Error('HTTP ' + r.status); return r.text(); })
        .then(txt => {
          let handled = false;
          const t = txt.trim();
          if (t.startsWith('{')) {
            try {
              const o = JSON.parse(t);
              if (o && o.cursor !== undefined) {
                rtState.pollCursor = o.cursor;
                (o.data || []).forEach(rtIngest);
                handled = true;
              }
            } catch (err) { /* 일반 형식으로 처리 */ }
          }
          if (!handled) rtParseMessage(txt).forEach(rtIngest);
          rtScheduleRender();
        })
        .catch(err => { rtState.lastError = err.message; });
    };
    poll();
    rtState.pollTimer = setInterval(poll, 250);
  }
}

// ==================== 데모(시뮬레이션) 모드 ====================
// 실제 RF 로그(telemetry_20260801_095829.csv)의 통계를 본떠 생성합니다:
//  · FAST/SLOW 두 종류를 약 4.2Hz로 번갈아 송신
//  · RSSI는 거리에 따라 -55 ~ -118 dBm 사이를 서서히 이동
//  · SNR은 RSSI와 상관 0.69로 연동, 링크가 나빠지면 패킷을 실제로 누락시킴

function rtStartDemo() {
  rtDisconnect(true);
  rtState.mode = 'demo';
  rtState.manualClose = false;
  rtSetConnState(true, '데모 중지');
  if (rtEl['rt-signal-reason']) rtEl['rt-signal-reason'].textContent = '데모 데이터 수신 중';

  let seqF = 0, seqS = 0, k = 0;
  let rpm = 1200, gear = 1, tps = 0, vss = 0, clt = 72, fuel = 0;

  rtState.demoTimer = setInterval(() => {
    k++;
    const phase = (k % 400) / 400;             // 가상의 랩 진행도
    const corner = Math.sin(phase * Math.PI * 6);

    tps = Math.max(0, Math.min(100, 55 + 45 * Math.sin(phase * Math.PI * 6 + 0.6) + (Math.random() - 0.5) * 6));
    rpm = Math.max(900, Math.min(9200, 3200 + 5200 * (tps / 100) + 700 * Math.sin(phase * Math.PI * 12)));
    gear = Math.max(1, Math.min(4, Math.round(1 + 3 * (tps / 100))));
    vss = Math.max(0, 20 + 85 * (tps / 100) + 8 * corner);
    clt = Math.max(60, Math.min(103, clt + (tps > 60 ? 0.02 : -0.015)));
    fuel += 0.0006 * (0.3 + tps / 100);

    // 링크 품질: 코스 반대편에서 멀어지며 악화
    const dist = 0.5 + 0.5 * Math.sin(phase * Math.PI * 2);
    const rssi = Math.round(-55 - 62 * dist + (Math.random() - 0.5) * 5);
    let snr = 10.5 - 17 * Math.max(0, dist - 0.55) / 0.45 + (Math.random() - 0.5) * 1.2;
    snr = Math.max(-9.5, Math.min(12.5, snr));

    // 링크가 나쁘면 실제로 패킷을 떨어뜨림 (seq는 증가시켜 손실이 검출되게)
    const lossProb = snr >= 8 ? 0.01 : snr >= 6 ? 0.04 : snr >= 3 ? 0.10 : snr >= 0 ? 0.22 : 0.40;

    if (k % 2) {
      seqF = (seqF + 1) & 0xff;
      if (Math.random() > lossProb) {
        rtIngest({ msg_type: 'FAST', seq: seqF, rpm: Math.round(rpm), tps_pct: +tps.toFixed(1), vss_kmh: Math.round(vss), rssi, snr: +snr.toFixed(1) });
      }
    } else {
      seqS = (seqS + 1) & 0xff;
      if (Math.random() > lossProb) {
        rtIngest({ msg_type: 'SLOW', seq: seqS, gear, clt_c: Math.round(clt), batt_v: +(13.9 + 0.5 * Math.sin(k / 90)).toFixed(2), fuel_used_l: +fuel.toFixed(2), rssi, snr: +snr.toFixed(1) });
      }
    }
    rtScheduleRender();
  }, 118);
}

// ==================== 초기화 ====================

function rtInit() {
  rtCacheDom();
  if (!rtEl['rt-grid']) return;

  try {
    const saved = localStorage.getItem('nssur_rt_url');
    if (saved && rtEl['rt-url']) rtEl['rt-url'].value = saved;
  } catch (e) { /* ignore */ }

  if (rtEl['rt-connect']) {
    rtEl['rt-connect'].addEventListener('click', () => {
      if (rtState.connected) rtDisconnect(true);
      else rtConnect(rtEl['rt-url'].value.trim(), false);
    });
  }
  if (rtEl['rt-url']) {
    rtEl['rt-url'].addEventListener('keydown', e => {
      if (e.key === 'Enter' && !rtState.connected) rtEl['rt-connect'].click();
    });
  }
  if (rtEl['rt-demo']) {
    rtEl['rt-demo'].addEventListener('click', () => {
      if (rtState.mode === 'demo' && rtState.connected) rtDisconnect(true);
      else rtStartDemo();
    });
  }

  // 수신이 끊겨도 경과 시간/등급이 갱신되도록 주기적으로 다시 그림
  setInterval(rtScheduleRender, 400);
  rtScheduleRender();
}
