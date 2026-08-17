/* NS26F Telemetry — 서스펜션 변위 보정 (표시값 = raw × 계수 + offset) */

const SUSPENSION_SCALE = Object.freeze({
  fl: 0.012940169,
  rl: 0.05229515,
  fr: 0.013051742,
  rr: 0.052189024
});
const SUSPENSION_OFFSET_DEFAULT = Object.freeze({ fl: 0, rl: 0, fr: 0, rr: 0 });
const SUSPENSION_CAL_STORE_KEY = 'nssur_suspension_offsets';
const suspensionOffsets = Object.assign({}, SUSPENSION_OFFSET_DEFAULT);

(function loadSuspensionOffsets() {
  try {
    const saved = JSON.parse(localStorage.getItem(SUSPENSION_CAL_STORE_KEY) || 'null');
    if (!saved) return;
    Object.keys(suspensionOffsets).forEach(wheel => {
      const value = Number(saved[wheel]);
      if (Number.isFinite(value)) suspensionOffsets[wheel] = value;
    });
  } catch (err) { /* 저장값이 깨졌으면 기본값 사용 */ }
})();

function getCalibratedSuspension(wheel, raw) {
  const value = Number(raw);
  if (!Number.isFinite(value) || !SUSPENSION_SCALE[wheel]) return NaN;
  return value * SUSPENSION_SCALE[wheel] + suspensionOffsets[wheel];
}

function saveSuspensionOffsets() {
  try { localStorage.setItem(SUSPENSION_CAL_STORE_KEY, JSON.stringify(suspensionOffsets)); } catch (err) { /* ignore */ }
}

let suspensionMenuEl = null;

function closeSuspensionPanel() {
  if (suspensionMenuEl) { suspensionMenuEl.remove(); suspensionMenuEl = null; }
}

function currentSuspensionRaw(wheel) {
  if (typeof activeSampledData === 'undefined' || !activeSampledData.length) return null;
  const row = activeSampledData[currentCursorIndex];
  if (!row) return null;
  const value = Number(row[`suspension_${wheel}_raw`]);
  return Number.isFinite(value) ? value : null;
}

function applySuspensionOffsetChange() {
  saveSuspensionOffsets();
  if (typeof rebuildRawChannel === 'function' && typeof globalData !== 'undefined' && globalData.length) {
    ['fl', 'rl', 'fr', 'rr'].forEach(wheel => rebuildRawChannel(`sus_${wheel}`, globalData));
  }
  if (typeof refreshChartsAfterFilter === 'function') refreshChartsAfterFilter();
  updateSuspensionCalBadges();
}

function openSuspensionPanel(x, y, focusWheel) {
  closeSuspensionPanel();
  const el = document.createElement('div');
  el.className = 'filter-menu suspension-menu';
  suspensionMenuEl = el;
  document.body.appendChild(el);

  const rows = ['fl', 'fr', 'rl', 'rr'].map(wheel => {
    const raw = currentSuspensionRaw(wheel);
    const calibrated = raw === null ? null : getCalibratedSuspension(wheel, raw);
    return `<div class="suspension-cal-row ${wheel === focusWheel ? 'is-focus' : ''}">
      <strong>${wheel.toUpperCase()}</strong>
      <span class="suspension-scale">raw × ${SUSPENSION_SCALE[wheel]}</span>
      <label>오프셋
        <input class="fm-num suspension-offset-input" type="number" step="0.01" data-wheel="${wheel}" value="${suspensionOffsets[wheel].toFixed(2)}">
        <span>mm</span>
      </label>
      <span class="suspension-live">${raw === null ? 'CSV를 불러오면 현재값 표시' : `raw ${raw.toFixed(0)} → ${calibrated.toFixed(2)} mm`}</span>
    </div>`;
  }).join('');

  el.innerHTML = `<div class="fm-head">
      <span class="fm-title">서스펜션 오프셋 보정</span>
      <button class="fm-close" data-act="close">✕</button>
    </div>
    <div class="fm-sec suspension-cal-grid">${rows}</div>
    <div class="fm-foot">
      <button class="fm-btn" data-act="reset">모든 오프셋 0으로</button>
    </div>
    <div class="fm-hint">표시값 = raw × 바퀴별 고정 계수 + 오프셋. 입력값은 이 브라우저에 저장되며 원본 CSV는 바뀌지 않습니다.</div>`;

  let timer = null;
  el.querySelector('[data-act="close"]').onclick = closeSuspensionPanel;
  el.querySelector('[data-act="reset"]').onclick = () => {
    Object.assign(suspensionOffsets, SUSPENSION_OFFSET_DEFAULT);
    applySuspensionOffsetChange();
    openSuspensionPanel(x, y, focusWheel);
  };
  el.querySelectorAll('.suspension-offset-input').forEach(input => {
    input.oninput = event => {
      const value = Number(event.target.value);
      if (!Number.isFinite(value)) return;
      suspensionOffsets[event.target.dataset.wheel] = value;
      clearTimeout(timer);
      timer = setTimeout(applySuspensionOffsetChange, 80);
    };
  });

  const rect = el.getBoundingClientRect();
  let left = x, top = y;
  if (left + rect.width > window.innerWidth - 8) left = window.innerWidth - rect.width - 8;
  if (top + rect.height > window.innerHeight - 8) top = Math.max(8, window.innerHeight - rect.height - 8);
  el.style.left = Math.max(8, left) + 'px';
  el.style.top = Math.max(8, top) + 'px';
  const focused = el.querySelector(`[data-wheel="${focusWheel}"]`);
  if (focused) focused.focus();
}

function updateSuspensionCalBadges() {
  document.querySelectorAll('.suspension-cal-trigger').forEach(button => {
    const wheel = button.dataset.wheel;
    const custom = Math.abs(suspensionOffsets[wheel]) > 1e-12;
    button.classList.toggle('cal-custom', custom);
    button.title = `${wheel.toUpperCase()} 오프셋 ${suspensionOffsets[wheel].toFixed(2)} mm — 클릭하여 변경`;
  });
}

function initSuspensionCalibration() {
  document.querySelectorAll('.suspension-cal-trigger').forEach(button => {
    button.addEventListener('click', event => {
      event.preventDefault();
      event.stopPropagation();
      const rect = button.getBoundingClientRect();
      openSuspensionPanel(rect.right - 430, rect.bottom + 8, button.dataset.wheel);
    });
  });
  updateSuspensionCalBadges();
}

document.addEventListener('click', event => {
  if (suspensionMenuEl && !suspensionMenuEl.contains(event.target) && !event.target.closest('.suspension-cal-trigger')) closeSuspensionPanel();
});
document.addEventListener('keydown', event => { if (event.key === 'Escape') closeSuspensionPanel(); });
